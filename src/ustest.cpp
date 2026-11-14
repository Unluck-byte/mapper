// Простой тест HC-SR04: pio run -e ustest -t upload
#ifdef US_TEST
#ifndef STAGE
#define STAGE 0   // 0 — только pulseIn; 1 +hwBegin; 2 +tofBegin; 3 — опрос из hw.cpp
#endif
#include <Arduino.h>
#include "config.h"
#include "hw.h"
#include "tof.h"

static const int TRIG[2] = {PIN_USF_TRIG, PIN_USR_TRIG};
static const int ECHO[2] = {PIN_USF_ECHO, PIN_USR_ECHO};
static const char *NAME[2] = {"ПЕРЕД", "ЗАД"};

void setup() {
  Serial.begin(115200);
#if STAGE >= 1
  hwBegin();
#endif
#if STAGE >= 2
  tofBegin(TOF_MODE_SCAN);
#endif
  for (int i = 0; i < 2; i++) { pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW); pinMode(ECHO[i], INPUT); }
}

extern volatile uint32_t echoEdges[2];
void loop() {
#if STAGE >= 3
  uint32_t t = millis();
  while (millis() - t < 1000) hwUpdate();
  Serial.printf("[st3] usUpdate: перед=%u мм зад=%u мм  импульсов=%lu/%lu ок=%lu/%lu isr=%lu/%lu\n", S.usFront, S.usRear,
                (unsigned long)S.usPulses[0], (unsigned long)S.usPulses[1], (unsigned long)S.usHits[0], (unsigned long)S.usHits[1],
                (unsigned long)echoEdges[0], (unsigned long)echoEdges[1]);
  return;
#endif
  Serial.printf("[st%d] ", STAGE);
  for (int i = 0; i < 2; i++) {
    int before = digitalRead(ECHO[i]);
    digitalWrite(TRIG[i], HIGH); delayMicroseconds(20); digitalWrite(TRIG[i], LOW);
    unsigned long us = pulseIn(ECHO[i], HIGH, 60000);
    if (us == 0) Serial.printf("%s: нет импульса (Echo до замера=%d)   ", NAME[i], before);
    else if (us > 30000) Serial.printf("%s: эха нет, импульс %lu мкс   ", NAME[i], us);
    else Serial.printf("%s: %.1f см (%lu мкс)   ", NAME[i], us * 0.0343f / 2, us);
    delay(60);
  }
  Serial.println();
}
#endif
