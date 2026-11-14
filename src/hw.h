// Низкоуровневое железо: моторы, энкодеры, гироскоп, ультразвук, ИК пола, батарея, серва, LED.
#pragma once
#include <Arduino.h>

enum Wheel { FL = 0, FR = 1, RL = 2, RR = 3 };

struct Sensors {
  // гироскоп
  bool  imuOk = false;
  float yaw = 0;          // рад, интеграл гироскопа (против часовой — плюс)
  float yawRate = 0;      // рад/с
  float gyroBiasDps = 0;
  // ультразвук, мм (US_MAX_MM = нет эха)
  uint16_t usFront = 0, usRear = 0;
  uint32_t usFrontMs = 0, usRearMs = 0;
  uint32_t usPulses[2] = {0, 0};   // диагностика: импульсов ECHO (датчик жив)
  uint32_t usHits[2] = {0, 0};     // из них с нормальным расстоянием
  // ИК пола: true — под датчиком пропасть
  bool cliff = false;
  // батарея, мВ (0 — не подключена)
  uint16_t batMv = 0;
};
extern Sensors S;

void hwBegin();
void hwUpdate();
void usReinit();                         // вызвать после tofBegin() (UART1 трогает GPIO15/16)                          // вызывать в loop() как можно чаще

// Моторы: -1..1 (знак — направление), уже с учётом инверсии
void motorsSet(float left, float right);
void motorsStop();
float motorLeft();
float motorRight();

// Энкодеры: число фронтов с прошлого вызова (всегда >= 0) и общий счётчик
void encReadDeltas(int32_t out[4]);
extern int32_t encTotal[4];

// Гироскоп
void imuCalibrate(uint32_t ms);           // машинка должна стоять
void imuSetStationary(bool still);        // подсказка «стоим» — тихая подстройка смещения

// Серва
void servoDeg(float deg);                 // 0..180, 90 = вперёд
float servoGetDeg();
uint32_t servoMovedMs();                  // millis() последнего поворота

// RGB-светодиод статуса
void ledSet(uint8_t r, uint8_t g, uint8_t b);
