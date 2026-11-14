#include "hw.h"
#include "config.h"
#include <Wire.h>
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_timer.h"

Sensors S;
int32_t encTotal[4] = {0, 0, 0, 0};

// ============================== Моторы (MX1508) ==============================
static const uint8_t PWM_BITS = 10;
static float mL = 0, mR = 0;

static void sideWrite(int pa, int pb, float u) {
  uint32_t d = (uint32_t)(fabsf(u) * ((1 << PWM_BITS) - 1));
  if (u > 0)      { ledcWrite(pa, d); ledcWrite(pb, 0); }
  else if (u < 0) { ledcWrite(pa, 0); ledcWrite(pb, d); }
  else            { ledcWrite(pa, 0); ledcWrite(pb, 0); }   // свободный ход
}

void motorsSet(float l, float r) {
  l = constrain(l, -1.0f, 1.0f);
  r = constrain(r, -1.0f, 1.0f);
  mL = l;
  mR = r;
  sideWrite(PIN_MOT_L_A, PIN_MOT_L_B, MOTOR_L_INVERT ? -l : l);
  sideWrite(PIN_MOT_R_A, PIN_MOT_R_B, MOTOR_R_INVERT ? -r : r);
}
void motorsStop() { motorsSet(0, 0); }
float motorLeft() { return mL; }
float motorRight() { return mR; }

// ============================== Энкодеры (PCNT) ==============================
static pcnt_unit_handle_t pcnt[4];
static const int ENC_PINS[4] = {PIN_ENC_FL, PIN_ENC_FR, PIN_ENC_RL, PIN_ENC_RR};

static void encBegin() {
  for (int i = 0; i < 4; i++) {
    pcnt_unit_config_t uc = {};
    uc.low_limit = -32768;
    uc.high_limit = 32767;
    ESP_ERROR_CHECK(pcnt_new_unit(&uc, &pcnt[i]));
    pcnt_glitch_filter_config_t fc = {};
    fc.max_glitch_ns = 10000;   // фильтр дребезга LM393
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt[i], &fc));
    pcnt_chan_config_t cc = {};
    cc.edge_gpio_num = ENC_PINS[i];
    cc.level_gpio_num = -1;
    pcnt_channel_handle_t ch;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt[i], &cc, &ch));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    gpio_pullup_en((gpio_num_t)ENC_PINS[i]);
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt[i]));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt[i]));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt[i]));
  }
}

void encReadDeltas(int32_t out[4]) {
  for (int i = 0; i < 4; i++) {
    int c = 0;
    pcnt_unit_get_count(pcnt[i], &c);
    pcnt_unit_clear_count(pcnt[i]);
    out[i] = c;
    encTotal[i] += c;
  }
}

// ============================== Гироскоп MPU6050 ==============================
static const uint8_t MPU = 0x68;
static uint32_t imuLastUs = 0;
static bool stationary = false;

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuGyroZ(int16_t &gz) {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)2) != 2) return false;
  gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

static void imuBegin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  S.imuOk = mpuWrite(0x6B, 0x00)     // проснуться
         && mpuWrite(0x1A, 0x03)     // DLPF 44 Гц
         && mpuWrite(0x1B, 0x08);    // ±500 °/с, 65.5 LSB/(°/с) (±250 упирался на быстрых поворотах)
  imuLastUs = micros();
}

void imuCalibrate(uint32_t ms) {
  if (!S.imuOk) return;
  double sum = 0;
  int n = 0;
  uint32_t t = millis();
  while (millis() - t < ms) {
    int16_t gz;
    if (mpuGyroZ(gz)) { sum += gz / 65.5; n++; }
    delay(4);
  }
  if (n) S.gyroBiasDps = sum / n;
  imuLastUs = micros();
}

void imuSetStationary(bool still) { stationary = still; }

static void imuUpdate() {
  if (!S.imuOk) return;
  uint32_t now = micros();
  if (now - imuLastUs < 4000) return;          // 250 Гц
  int16_t gz;
  if (!mpuGyroZ(gz)) return;
  float dt = (now - imuLastUs) * 1e-6f;
  imuLastUs = now;
  float dps = gz / 65.5f;
  // стоим — медленно подстраиваем смещение нуля (MPU6050 «уплывает»)
  // стоим (моторы выключены) — подстраиваем ноль. После включения питания моторов/сервы ноль MPU6050
  // уезжает на несколько °/с, поэтому ловим ошибку до 8 °/с (τ≈1 с), а «вращение» в покое обнуляем.
  // Быстрее 8 °/с — значит машинку реально крутят руками: это учитываем.
  float err = dps - S.gyroBiasDps;
  bool still = stationary && fabsf(err) < 8.0f;
  if (still) S.gyroBiasDps += 0.004f * err;
  float rate = still ? 0 : (dps - S.gyroBiasDps) * GYRO_SIGN * DEG_TO_RAD;
  S.yawRate = rate;
  S.yaw += rate * dt;
}

// ============================== Ультразвук HC-SR04 ==============================
static volatile uint32_t echoStart[2], echoDur[2];
static volatile bool echoDone[2];
static const int ECHO_PIN[2] = {PIN_USF_ECHO, PIN_USR_ECHO};
static const int TRIG_PIN[2] = {PIN_USF_TRIG, PIN_USR_TRIG};

volatile uint32_t echoEdges[2];
static void IRAM_ATTR echoIsr(void *arg) {
  int i = (int)(intptr_t)arg;
  echoEdges[i]++;
  uint32_t t = (uint32_t)esp_timer_get_time();
  if (gpio_get_level((gpio_num_t)ECHO_PIN[i])) echoStart[i] = t;
  else if (echoStart[i]) { echoDur[i] = t - echoStart[i]; echoStart[i] = 0; echoDone[i] = true; }
}

void usReinit();
static void usBegin() {
  for (int i = 0; i < 2; i++) {
    pinMode(TRIG_PIN[i], OUTPUT);
    digitalWrite(TRIG_PIN[i], LOW);
    gpio_config_t gc = {};
    gc.pin_bit_mask = 1ULL << ECHO_PIN[i];
    gc.mode = GPIO_MODE_INPUT;
    gc.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&gc);
    static bool svc = false;
    if (!svc) { gpio_install_isr_service(ESP_INTR_FLAG_IRAM); svc = true; }
    gpio_isr_handler_remove((gpio_num_t)ECHO_PIN[i]);
    gpio_isr_handler_add((gpio_num_t)ECHO_PIN[i], echoIsr, (void *)(intptr_t)i);
  }
}

// UART1 (TOF) на S3 по умолчанию сидит на GPIO15/16 и при запуске перенастраивает их —
// после tofBegin() выводы ультразвука настраиваем заново.
void usReinit() { usBegin(); }

static void usUpdate() {
  // по очереди: перед, зад — чтобы не ловить чужое эхо
  static uint32_t last = 0;
  static int cur = 0;
  static bool started = false;
  uint32_t now = millis();
  if (now - last < US_SLOT_MS) return;
  last = now;
  if (started) {
    uint16_t mm = US_MAX_MM;
    if (echoDone[cur]) {
      uint32_t d = echoDur[cur];
      S.usPulses[cur]++;
      if (d > 100 && d < 30000) { mm = min<uint32_t>(d * 343 / 2000, US_MAX_MM); S.usHits[cur]++; }
    }
    if (cur == 0) { S.usFront = mm; S.usFrontMs = now; }
    else          { S.usRear = mm;  S.usRearMs = now; }
    cur ^= 1;
  }
  echoDone[cur] = false;
  digitalWrite(TRIG_PIN[cur], HIGH);
  delayMicroseconds(20);   // 10 мкс мало: шифтер HW-209 затягивает фронт
  digitalWrite(TRIG_PIN[cur], LOW);
  started = true;
}

// ============================== ИК пола, батарея ==============================
static void cliffUpdate() {
  static uint32_t last = 0;
  static uint8_t hits = 0;
  if (millis() - last < 5) return;
  last = millis();
  bool c = digitalRead(PIN_CLIFF) == CLIFF_ACTIVE;
  hits = c ? min(hits + 1, 10) : 0;
  S.cliff = hits >= 3;
}

static void batUpdate() {
  static uint32_t last = 0;
  static float f = 0;
  if (millis() - last < 200) return;
  last = millis();
  float mv = analogReadMilliVolts(PIN_BAT) * BAT_DIV_RATIO;
  f = (f == 0) ? mv : f + 0.2f * (mv - f);
  S.batMv = f < BAT_PRESENT_MV ? 0 : (uint16_t)f;
}

// ============================== Серва ==============================
static float servoCur = 90;
static uint32_t servoMoved = 0;

void servoDeg(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  float us = SERVO_US_MIN + deg / 180.0f * (SERVO_US_MAX - SERVO_US_MIN);
  ledcWrite(PIN_SERVO, (uint32_t)(us * ((1 << 14) - 1) / 20000.0f));
  if (fabsf(deg - servoCur) > 0.1f) servoMoved = millis();
  servoCur = deg;
}
float servoGetDeg() { return servoCur; }
uint32_t servoMovedMs() { return servoMoved; }

// ============================== LED ==============================
void ledSet(uint8_t r, uint8_t g, uint8_t b) { rgbLedWrite(PIN_LED, r, g, b); }

// ============================== общее ==============================
void hwBegin() {
  for (int p : {PIN_MOT_L_A, PIN_MOT_L_B, PIN_MOT_R_A, PIN_MOT_R_B}) {
    ledcAttach(p, MOTOR_PWM_FREQ, PWM_BITS);
    ledcWrite(p, 0);
  }
  ledcAttach(PIN_SERVO, 50, 14);
  servoDeg(90);
  encBegin();
  imuBegin();
  usBegin();
  pinMode(PIN_CLIFF, INPUT_PULLUP);
  analogSetPinAttenuation(PIN_BAT, ADC_11db);
}

void hwUpdate() {
  imuUpdate();
  usUpdate();
  cliffUpdate();
  batUpdate();
}
