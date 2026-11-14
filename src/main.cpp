// mapper — SLAM-машинка на ESP32-S3.
// Прошивка: датчики, моторы, защита, скан, связь с компом (pc/). Схема: docs/wiring.png
#include <Arduino.h>
#include "config.h"
#include "hw.h"
#include "tof.h"
#include "robot.h"
#include "net.h"
#include "cli.h"
#include <WiFi.h>

static void ledStatus() {
  static uint32_t last = 0;
  if (millis() - last < 250) return;
  last = millis();
  bool blink = (millis() / 250) % 2;
  if (*R.block)                 ledSet(blink ? 40 : 0, 0, blink ? 40 : 0);   // фиолетовый мигает — защита
  else if (R.mode == M_SCAN)    ledSet(0, 0, 40);                            // синий — скан
  else if (netPcConnected())    ledSet(0, 30, 0);                            // зелёный — связь с компом
  else if (WiFi.status() == WL_CONNECTED) ledSet(30, 20, 0);                 // жёлтый — Wi-Fi есть, компа нет
  else                          ledSet(blink ? 40 : 0, 0, 0);                // красный мигает — нет Wi-Fi
}

#ifdef DEBUG_BOOT
#define DBG(x) do { Serial.printf("[boot %lu] %s\n", millis(), x); Serial.flush(); delay(30); } while (0)
#else
#define DBG(x)
#endif

void setup() {
  Serial.begin(115200);
#ifdef DEBUG_BOOT
  delay(4000);
#endif
  DBG("start");
  hwBegin();
  DBG("hw ok");
  ledSet(40, 0, 0);
  delay(800);
  Serial.println("\nmapper: калибровка гироскопа — не трогайте машинку...");
  imuCalibrate(1500);
  Serial.printf("гироскоп: %s, смещение %.3f °/с\n", S.imuOk ? "ok" : "НЕ НАЙДЕН", S.gyroBiasDps);
  bool tof = tofBegin(TOF_MODE_SCAN);
  Serial.printf("TOF400F: %s\n", tof ? "ok" : "НЕ ОТВЕЧАЕТ");
  DBG("tof done");
  robotBegin();
  DBG("robot ok");
  netBegin();
  DBG("net ok");
  cliBegin();
  DBG("cli ok");
}

void loop() {
#ifdef DEBUG_BOOT
  static uint32_t hb = 0; if (millis() - hb > 2000) { hb = millis(); Serial.printf("[loop %lu]\n", hb); }
#endif
  hwUpdate();
  tofUpdate();
  robotUpdate();
  netUpdate();
  cliUpdate();
  ledStatus();
}
