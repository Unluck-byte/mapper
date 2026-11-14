// Логика машинки: одометрия, регуляторы, ручная езда, маршрут, скан, защита.
#pragma once
#include <Arduino.h>

enum Mode : uint8_t { M_IDLE, M_MANUAL, M_PATH, M_TURN, M_SCAN, M_TEST };

struct ScanPoint {
  int16_t thMrad;    // курс машинки в момент замера, мрад
  int16_t relDeci;   // угол сервы относительно «вперёд», 0.1° (+ влево)
  uint16_t mm;       // расстояние от датчика, 0 — нет цели
};

struct RobotState {
  float x = 0, y = 0, th = 0;   // мм, мм, рад (против часовой — плюс)
  float vL = 0, vR = 0;         // измеренные скорости сторон, мм/с
  float v = 0, w = 0;           // заданные после защиты
  Mode mode = M_IDLE;
  const char *block = "";       // "" | front | rear | cliff | bat | stall
  int wpIdx = 0, wpCount = 0;
  uint32_t taskId = 0;
};
extern RobotState R;

void robotBegin();
void robotUpdate();                                      // в loop()

void robotManual(float v, float w);                     // мм/с, рад/с; повторять < CMD_TIMEOUT_MS
void robotStop(const char *why);
bool robotPath(const float *xy, int n, uint32_t id);    // точки x0,y0,x1,y1... в мм
void robotTurnTo(float th, uint32_t id);
void robotScan(uint32_t id);
void robotSetPose(float x, float y, float th);
void robotTestMotors(float l, float r, uint32_t ms);    // сырой ШИМ без регуляторов (тест)

// готовый скан (после события "scan_done")
const ScanPoint *robotScanPoints(int &n, float &x, float &y);

// события для сети/консоли: JSON-строки
bool robotPopEvent(char *buf, size_t len);
const char *modeName(Mode m);
