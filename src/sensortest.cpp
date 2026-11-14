// Тест датчиков по Wi-Fi (без компа): ESP поднимает свою сеть, страница с показаниями.
// Сборка: pio run -e sensortest -t upload
// Подключиться телефоном к Wi-Fi "mapper-test" (пароль 12345678) → http://192.168.4.1
#ifdef SENSOR_TEST
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "hw.h"
#include "tof.h"

extern volatile uint32_t echoEdges[2];
static const char *AP_SSID = "mapper-test";
static const char *AP_PASS = "12345678";

WebServer web(80);

// акселерометр/температура (гироскоп Z читает hw.cpp)
static float ax, ay, az, tempC;
static bool accOk = false;
static uint32_t accMs = 0;

// TOF: частота замеров
static uint32_t tofSeqPrev = 0, tofRateMs = 0;
static float tofHz = 0;

// серва: 0 — вручную, 1 — качается 0..180, 2 — следует за расстоянием
static int servoMode = 1;
static float sweepDeg = 90, sweepDir = 1;
static uint32_t servoMs = 0;

// TOF: поиск подключения (прямо / RX-TX перепутаны, разные скорости)
static String tofWire = "ищу…";
static uint32_t tofProbeMs = 0;
static bool tofProbe() {
  const int pins[2][2] = {{PIN_TOF_RX, PIN_TOF_TX}, {PIN_TOF_TX, PIN_TOF_RX}};
  const uint32_t bauds[] = {115200, 9600, 921600, 230400, 460800, 38400, 57600, 19200};
  for (int p = 0; p < 2; p++)
    for (uint32_t b : bauds) {
      if (tofBeginOn(pins[p][0], pins[p][1], b, TOF_MODE_SCAN, 2)) {
        tofWire = String(p == 0 ? "по схеме" : "RX/TX ПЕРЕПУТАНЫ (работает, но поменяй провода)") +
                  ", " + b + " бод";
        if (b != 115200) tofWire += " (НЕ стандартная скорость)";
        return true;
      }
      web.handleClient();
    }
  tofBeginOn(PIN_TOF_RX, PIN_TOF_TX, 115200, TOF_MODE_SCAN);
  tofWire = "НЕ НАЙДЕН ни на GPIO5/6, ни перекрёстно — проверь питание VIN/GND и провода";
  return false;
}

static void readAccel() {
  if (millis() - accMs < 20) return;
  accMs = millis();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((uint8_t)0x68, (uint8_t)8) != 8) {
    accOk = false;
    return;
  }
  int16_t r[4];
  for (int i = 0; i < 4; i++) r[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  ax = r[0] / 16384.0f; ay = r[1] / 16384.0f; az = r[2] / 16384.0f;   // ±2g
  tempC = r[3] / 340.0f + 36.53f;
  accOk = true;
}

static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>mapper: датчики</title>
<style>
:root{--bg:#111;--card:#1c1c1f;--fg:#eee;--dim:#888;--ok:#3c6;--bad:#e44;--acc:#4af}
body{margin:0;background:var(--bg);color:var(--fg);font:16px system-ui,sans-serif;padding:12px}
h1{font-size:18px;margin:0 0 10px}.card{background:var(--card);border-radius:12px;padding:12px;margin-bottom:10px}
.big{font-size:44px;font-weight:700;font-variant-numeric:tabular-nums}.u{font-size:18px;color:var(--dim)}
.row{display:flex;justify-content:space-between;gap:8px;font-variant-numeric:tabular-nums}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px;background:var(--bad)}
.dot.ok{background:var(--ok)}.bar{height:8px;background:#333;border-radius:4px;overflow:hidden;margin-top:6px}
.bar i{display:block;height:100%;background:var(--acc)}button{background:#333;color:var(--fg);border:0;border-radius:8px;padding:10px 12px;font-size:15px;margin:4px 4px 0 0}
button.on{background:var(--acc);color:#000}input[type=range]{width:100%;margin-top:10px}.dim{color:var(--dim);font-size:13px}
</style></head><body>
<h1>mapper — тест датчиков <span class="dim">v3</span> <span id="link" class="dim"></span></h1>
<div class="card"><div><span id="tofd" class="dot"></span>TOF400F <span class="dim" id="tofinfo"></span></div>
<div><span class="big" id="tof">—</span> <span class="u">см</span></div><div class="bar"><i id="tofbar" style="width:0"></i></div><div class="dim" id="tofwire"></div></div>
<div class="card"><div><span id="imud" class="dot"></span>MPU6050 <span class="dim" id="temp"></span></div>
<div class="row"><span>Курс (гироскоп)</span><b id="yaw">—</b></div>
<div class="row"><span>Скорость поворота</span><b id="rate">—</b></div>
<div class="row"><span>Ускорение X / Y / Z</span><b id="acc">—</b></div>
<div class="row"><span>Наклон: крен / тангаж</span><b id="tilt">—</b></div>
<button onclick="cmd('zero')">Курс = 0</button><button onclick="cmd('calib')">Калибровка (не двигать 2 с)</button></div>
<div class="card"><div>Ультразвук HC-SR04 <span class="dim">(эхо каждые 70 мс)</span></div>
<div class="row" style="margin-top:6px"><div><span id="ufd" class="dot"></span>Перед<div><span class="big" id="uf">—</span> <span class="u">см</span></div><div class="dim" id="ufi"></div></div>
<div style="text-align:right"><span id="urd" class="dot"></span>Зад<div><span class="big" id="ur">—</span> <span class="u">см</span></div><div class="dim" id="uri"></div></div></div></div>
<div class="card"><div>Серва: <b id="sv">—</b>°</div>
<button id="m1" onclick="cmd('mode?m=1')">Качать 0–180</button><button id="m2" onclick="cmd('mode?m=2')">По расстоянию</button><button id="m0" onclick="cmd('mode?m=0')">Вручную</button>
<input type="range" id="sl" min="0" max="180" value="90" oninput="cmd('servo?deg='+this.value)">
<div class="dim">90° — вперёд, 180° — влево. По расстоянию: близко 0°, 2 м и дальше 180°.</div></div>
<div class="card dim" id="other"></div>
<script>
const $=id=>document.getElementById(id);let fails=0;
const hist={f:[],r:[]};
function usShow(k,mm,p,h,lv){const now=Date.now(),H=hist[k];H.push({p,h,t:now});
while(H.length>2&&now-H[0].t>1500)H.shift();const o=H[0];if(now-o.t<800)return;
const dt=(now-o.t)/1000,pr=(p-o.p)/dt,hr=(h-o.h)/dt,el=$('u'+k);
let st;if(pr<1){el.textContent='—';st=lv?'на Echo постоянно 1: датчик не тянет линию — нет 5В/GND на датчике или Echo не на том канале шифтера':'Echo в 0, но импульсов нет: не доходит Trig (GPIO→A, B→Trig, тот же номер канала)'}
else if(hr<1){el.textContent='> 300';st='датчик жив, но эха нет (пусто впереди или далеко)'}
else{el.textContent=(mm/10).toFixed(1);st='ок, '+hr.toFixed(0)+' зам/с'}
$('u'+k+'d').className='dot'+(pr>=1?' ok':'');$('u'+k+'i').textContent=st}
function cmd(c){fetch('/'+c).catch(()=>{})}
async function tick(){try{const r=await fetch('/data?'+Date.now(),{cache:'no-store'});const d=await r.json();fails=0;
$('link').textContent='● связь';$('link').style.color='#3c6';
$('tofd').className='dot'+(d.tofok?' ok':'');$('tof').textContent=d.tof?(d.tof/10).toFixed(1):'нет цели';
$('tofbar').style.width=Math.min(100,d.tof/40)+'%';$('tofinfo').textContent=d.tofhz.toFixed(0)+' зам/с, режим '+d.tofmode;
$('tofwire').textContent='Подключение: '+d.tofwire+' · байт от датчика: '+d.tofrx;
$('imud').className='dot'+(d.imu&&d.acc?' ok':'');$('temp').textContent=d.acc?d.temp.toFixed(1)+' °C':'';
$('yaw').textContent=d.yaw.toFixed(1)+'°';$('rate').textContent=d.rate.toFixed(1)+' °/с';
$('acc').textContent=[d.ax,d.ay,d.az].map(v=>v.toFixed(2)).join(' / ')+' g';
const roll=Math.atan2(d.ay,d.az)*57.3,pitch=Math.atan2(-d.ax,Math.hypot(d.ay,d.az))*57.3;
$('tilt').textContent=roll.toFixed(1)+'° / '+pitch.toFixed(1)+'°';
$('sv').textContent=d.servo.toFixed(0);for(const m of [0,1,2])$('m'+m).className=d.smode==m?'on':'';
if(d.smode!=0)$('sl').value=d.servo;
usShow('f',d.usf,d.usfp,d.usfh,d.lvf);usShow('r',d.usr,d.usrp,d.usrh,d.lvr);
$('other').textContent='ИК пола '+(d.cliff?'ПРОПАСТЬ':'пол')+' · батарея '+(d.bat/1000).toFixed(2)+' В · смещение гиро '+d.bias.toFixed(2)+' °/с · работает '+d.up+' с'
}catch(e){if(++fails>3){$('link').textContent='● нет связи';$('link').style.color='#e44'}}
setTimeout(tick,100)}tick();
</script></body></html>)HTML";

static void sendData() {
  const TofSample &t = tofLast();
  char buf[768];
  snprintf(buf, sizeof(buf),
           "{\"tof\":%u,\"tofok\":%d,\"tofrx\":%lu,\"tofwire\":\"%s\",\"tofhz\":%.1f,\"tofmode\":%d,"
           "\"imu\":%d,\"acc\":%d,\"yaw\":%.2f,\"rate\":%.2f,\"bias\":%.3f,"
           "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"temp\":%.1f,"
           "\"servo\":%.0f,\"smode\":%d,\"usf\":%u,\"usr\":%u,\"usfp\":%lu,\"usfh\":%lu,\"usrp\":%lu,\"usrh\":%lu,\"lvf\":%d,\"lvr\":%d,\"cliff\":%d,\"bat\":%u,\"up\":%lu}",
           t.mm, tofOk() ? 1 : 0, (unsigned long)tofRxBytes(), tofWire.c_str(), tofHz, TOF_MODE_SCAN,
           S.imuOk ? 1 : 0, accOk ? 1 : 0, S.yaw * RAD_TO_DEG, S.yawRate * RAD_TO_DEG, S.gyroBiasDps,
           ax, ay, az, tempC,
           servoGetDeg(), servoMode, S.usFront, S.usRear, (unsigned long)S.usPulses[0], (unsigned long)S.usHits[0], (unsigned long)S.usPulses[1], (unsigned long)S.usHits[1], digitalRead(PIN_USF_ECHO), digitalRead(PIN_USR_ECHO), S.cliff ? 1 : 0, S.batMv,
           (unsigned long)(millis() / 1000));
  web.send(200, "application/json", buf);
}

void setup() {
  Serial.begin(115200);
  hwBegin();
  motorsStop();
  ledSet(0, 0, 40);
  imuSetStationary(true);
  imuCalibrate(1500);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.begin();

  web.on("/", [] { web.sendHeader("Cache-Control", "no-store"); web.send_P(200, "text/html; charset=utf-8", PAGE); });
  web.on("/data", sendData);
  web.on("/servo", [] { servoMode = 0; servoDeg(constrain(web.arg("deg").toFloat(), 0.f, 180.f)); web.send(200, "text/plain", "ok"); });
  web.on("/mode", [] { servoMode = web.arg("m").toInt(); web.send(200, "text/plain", "ok"); });
  web.on("/zero", [] { S.yaw = 0; web.send(200, "text/plain", "ok"); });
  web.on("/calib", [] { web.send(200, "text/plain", "ok"); imuCalibrate(2000); S.yaw = 0; });
  web.begin();
  bool tof = tofProbe();
  tofProbeMs = millis();

  Serial.printf("TOF %s, MPU %s. Wi-Fi \"%s\" / %s -> http://%s\n", tof ? "ok" : "НЕТ", S.imuOk ? "ok" : "НЕТ",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

static uint32_t loops = 0, maxLoop = 0;
void loop() {
  static uint32_t lastLoop = 0;
  { uint32_t n = millis(); if (lastLoop && n - lastLoop > maxLoop) maxLoop = n - lastLoop; lastLoop = n; loops++; }
  hwUpdate();
  tofUpdate();
  readAccel();
  web.handleClient();
  ArduinoOTA.handle();

  uint32_t now = millis();
  // датчик пропал / не найден — ищем заново каждые 15 с (можно переткнуть провода на ходу)
  if (!tofOk() && now - tofProbeMs > 15000) { tofProbe(); tofProbeMs = millis(); now = tofProbeMs; }
  if (now - tofRateMs >= 1000) {
    uint32_t s = tofLast().seq;
    tofHz = (s - tofSeqPrev) * 1000.0f / (now - tofRateMs);
    tofSeqPrev = s;
    tofRateMs = now;
  }

  if (now - servoMs >= 20) {
    servoMs = now;
    if (servoMode == 1) {
      sweepDeg += sweepDir * 2;   // 100 °/с
      if (sweepDeg >= 180) { sweepDeg = 180; sweepDir = -1; }
      if (sweepDeg <= 0) { sweepDeg = 0; sweepDir = 1; }
      servoDeg(sweepDeg);
    } else if (servoMode == 2) {
      // нули (нет цели) пропускаем — иначе серва прыгает на 180°; плавно, не быстрее 3° за шаг
      static float target = 90;
      uint16_t mm = tofLast().mm;
      if (mm) target = constrain(mm / 2000.0f * 180.0f, 0.f, 180.f);
      float d = constrain(0.15f * (target - servoGetDeg()), -3.f, 3.f);
      if (fabsf(d) > 0.2f) servoDeg(servoGetDeg() + d);
    }
  }

  // LED: зелёный — телефон подключён, синий — ждём; красный — нет TOF или MPU
  static uint32_t ledMs = 0;
  if (now - ledMs > 300) {
    ledMs = now;
    if (!tofOk() || !S.imuOk) ledSet(40, 0, 0);
    else if (WiFi.softAPgetStationNum()) ledSet(0, 40, 0);
    else ledSet(0, 0, 40);
  }

  static uint32_t prMs = 0;
  if (now - prMs > 500) {
    prMs = now;
    Serial.printf("tof=%u мм  yaw=%.1f°  acc=%.2f/%.2f/%.2f g  servo=%.0f  us=%u/%u  echo=%lu/%lu ok=%lu/%lu  pinEcho=%d/%d isr=%lu/%lu loop=%lu/с maxloop=%lu мс\n", tofLast().mm, S.yaw * RAD_TO_DEG, ax, ay, az, servoGetDeg(), S.usFront, S.usRear, (unsigned long)S.usPulses[0], (unsigned long)S.usPulses[1], (unsigned long)S.usHits[0], (unsigned long)S.usHits[1], digitalRead(PIN_USF_ECHO), digitalRead(PIN_USR_ECHO), (unsigned long)echoEdges[0], (unsigned long)echoEdges[1], (unsigned long)(loops * 2), (unsigned long)maxLoop);
    loops = 0; maxLoop = 0;
  }
}
#endif
