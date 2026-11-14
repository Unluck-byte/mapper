#include "tof.h"
#include "config.h"
#include "hw.h"

#define TOF_ADDR    1
#define REG_REBOOT  0x0001
#define REG_MODE    0x0004
#define REG_PERIOD  0x0005   // период измерения, мс
#define REG_DIST    0x0010

static TofSample last;
static uint32_t lastSend = 0;
static bool waiting = false;
static uint32_t rxBytes = 0;
uint32_t tofRxBytes() { return rxBytes; }

static uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
  }
  return c;
}

static void sendFrame(uint8_t fn, uint16_t reg, uint16_t val) {
  uint8_t q[8] = {TOF_ADDR, fn, (uint8_t)(reg >> 8), (uint8_t)reg, (uint8_t)(val >> 8), (uint8_t)val};
  uint16_t c = crc16(q, 6);
  q[6] = c & 0xFF;
  q[7] = c >> 8;
  Serial1.write(q, 8);
}

static bool transact(uint8_t fn, uint16_t reg, uint16_t val, uint8_t *r, size_t len) {
  while (Serial1.available()) Serial1.read();
  sendFrame(fn, reg, val);
  size_t n = 0;
  uint32_t t = millis();
  while (n < len && millis() - t < 300) {
    if (!Serial1.available()) continue;
    uint8_t b = Serial1.read();
    rxBytes++;
    if (n == 0 && b != TOF_ADDR) continue;
    if (n == 1 && b != fn) { n = (b == TOF_ADDR); continue; }
    r[n++] = b;
  }
  return n == len && crc16(r, len - 2) == (uint16_t)(r[len - 2] | (r[len - 1] << 8));
}

static bool readReg(uint16_t reg, uint16_t &v) {
  uint8_t r[7];
  if (!transact(0x03, reg, 1, r, 7) || r[2] != 2) return false;
  v = (r[3] << 8) | r[4];
  return true;
}

bool tofReadReg(uint16_t reg, uint16_t &v) { return readReg(reg, v); }
bool tofWriteReg(uint16_t reg, uint16_t v) { uint8_t r[8]; return transact(0x06, reg, v, r, 8); }

bool tofBegin(uint16_t mode) { return tofBeginOn(PIN_TOF_RX, PIN_TOF_TX, 115200, mode); }

bool tofBeginOn(int rx, int tx, uint32_t baud, uint16_t mode, int tries) {
  // ВАЖНО: end() до первого begin() отключает выводы UART1 по умолчанию (на S3 это GPIO15/16 =
  // Trig переднего и Echo заднего ультразвука) — поэтому только если порт уже был запущен.
  static bool begun = false;
  if (begun) Serial1.end();
  Serial1.begin(baud, SERIAL_8N1, rx, tx);
  begun = true;
  usReinit();
  Serial1.setRxFIFOFull(1);
  delay(300);
  uint16_t m = 0xFFFF;
  bool ok = false;
  for (int i = 0; i < tries && !(ok = readReg(REG_MODE, m)); i++) delay(50);
  if (!ok) return false;
  // период измерения (рег. 0x05): с завода 500 мс — серва за это время проходит 24°, и одно
  // расстояние размазывается дугой. Применяется после перезагрузки модуля, хранится в нём.
  uint16_t per = 0;
  readReg(REG_PERIOD, per);
  if (m != mode || per != TOF_PERIOD_MS) {
    uint8_t r[8];
    if (m != mode) transact(0x06, REG_MODE, mode, r, 8);
    if (per != TOF_PERIOD_MS) transact(0x06, REG_PERIOD, TOF_PERIOD_MS, r, 8);
    transact(0x06, REG_REBOOT, 0x1000, r, 8);
    delay(1500);
  }
  return true;
}

// разбор ответа "01 03 02 HH LL CRC CRC"
static bool parse(uint8_t b, uint16_t &d) {
  static uint8_t f[7];
  static uint8_t n = 0;
  if (n == 0 && b != TOF_ADDR) return false;
  if (n == 1 && b != 0x03) { n = (b == TOF_ADDR); return false; }
  if (n == 2 && b != 0x02) { n = (b == TOF_ADDR); return false; }
  f[n++] = b;
  if (n < 7) return false;
  n = 0;
  if (crc16(f, 5) != (uint16_t)(f[5] | (f[6] << 8))) return false;
  d = (f[3] << 8) | f[4];
  return true;
}

void tofUpdate() {
  uint32_t now = millis();
  if ((!waiting && now - lastSend >= 15) || (waiting && now - lastSend >= 80)) {
    sendFrame(0x03, REG_DIST, 1);
    lastSend = now;
    waiting = true;
  }
  uint16_t d;
  while (Serial1.available()) {
    rxBytes++;
    if (!parse(Serial1.read(), d)) continue;
    waiting = false;
    last.mm = (d == 0 || d > TOF_MAX_MM) ? 0 : d + TOF_OFFSET_MM;
    last.ms = now;
    last.seq++;
  }
}

const TofSample &tofLast() { return last; }
bool tofOk() { return last.ms && millis() - last.ms < 1000; }
