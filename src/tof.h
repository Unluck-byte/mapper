// TOF400F по UART, Modbus RTU 115200, адрес 1. Опрос без блокировки.
#pragma once
#include <Arduino.h>

struct TofSample {
  uint16_t mm = 0;      // 0 — нет цели / ошибка (с поправкой TOF_OFFSET_MM)
  uint32_t ms = 0;      // millis() получения
  uint32_t seq = 0;     // номер замера
};

bool tofBegin(uint16_t mode);   // true — датчик ответил
bool tofBeginOn(int rx, int tx, uint32_t baud, uint16_t mode, int tries = 5);   // для диагностики
bool tofReadReg(uint16_t reg, uint16_t &v);    // отладка: чтение/запись регистров модуля
bool tofWriteReg(uint16_t reg, uint16_t v);
uint32_t tofRxBytes();          // сколько байт пришло от датчика (диагностика)
void tofUpdate();               // вызывать в loop()
const TofSample &tofLast();
bool tofOk();                   // были ответы за последнюю секунду
