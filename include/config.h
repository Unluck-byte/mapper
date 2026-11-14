// Настройки машинки mapper. Пины — по схеме docs/wiring.png.
#pragma once
#include <Arduino.h>

// ============================== ПИНЫ ==============================
#define PIN_SERVO      4
#define PIN_TOF_TX     5    // ESP TX -> TOF RXD
#define PIN_TOF_RX     6    // ESP RX <- TOF TXD
#define PIN_USF_ECHO   7    // ультразвук перед (через преобразователь HW-209)
#define PIN_USF_TRIG   15
#define PIN_USR_ECHO   16   // ультразвук зад (через HW-209)
#define PIN_USR_TRIG   17
#define PIN_CLIFF      18   // ИК пола
#define PIN_MOT_L_A    10   // левая сторона: MX1508 IN1 (проверено на машинке)
#define PIN_MOT_L_B    11
#define PIN_MOT_R_A    8    // правая сторона: MX1508 IN3
#define PIN_MOT_R_B    9
#define PIN_ENC_FR     2    // GPIO12 не используется
#define PIN_ENC_FL     13
#define PIN_ENC_RL     14
#define PIN_ENC_RR     21
#define PIN_I2C_SDA    41
#define PIN_I2C_SCL    42
#define PIN_BAT        1    // делитель 100k/27k (по желанию; не установлен — замер выключен сам)
#define PIN_LED        48   // встроенный RGB (на DevKitC-1 v1.1 — 38)

// ======================= КАЛИБРОВКА (значения проверены на машинке 2026-11-06) =======================
// Моторы. Если сторона едет назад при команде «вперёд» — поставить true.
#define MOTOR_L_INVERT   true    // проверено тестом drivetest (WASD)
#define MOTOR_R_INVERT   false
#define MOTOR_PWM_FREQ   500     // замерено: на 10 кГц MX1508+TT трогаются только с 70%, на 500 Гц — с 25%
#define MOTOR_PWM_MIN    0.24f   // замер на полу (9 В): прямо трогается с 23% (~290 мм/с), поворот — с 24%.
                                 // Меньше этого ШИМ колесу не даём вообще — иначе стоит и пищит
#define MOTOR_PWM_MAX    0.75f   // питание моторов 9 В: 9 × 0.75 ≈ 6.7 В в пике (TT — до 6 В номинал)

// Колёса и энкодеры
#define WHEEL_DIAM_MM    65.0f
#define ENC_SLOTS        20      // щелей на диске
#define ENC_EDGES        2       // считаем оба фронта
#define MM_PER_EDGE      (PI * WHEEL_DIAM_MM / (ENC_SLOTS * ENC_EDGES))   // ≈5.1 мм
#define TRACK_MM         235.0f  // ЭФФЕКТИВНАЯ колея по замеру на полу (геометрическая 150: на месте колёса
                                 // проскальзывают, машинка поворачивается в ~1.6 раза медленнее расчёта)
#define ROBOT_RADIUS_MM  140      // радиус «круга», в который вписана машинка

// Скорости
#define MAX_SPEED_MM_S   450.0f  // предел скорости (джойстик), замер: 30% ≈ 465 мм/с
#define PATH_SPEED_MM_S  300.0f  // по маршруту — чуть выше порога трогания (~290 мм/с)
#define TURN_RATE_RAD_S  1.5f    // ≈86 °/с (замер: 28% ШИМ)
#define ACCEL_MM_S2      2000.0f // почти без плавного разгона: медленный набор = колёса стоят ниже порога

// Гироскоп. +1, если поворот против часовой даёт плюс (тест: imu).
#define GYRO_SIGN        1

// Серва: 90° = смотрит вперёд. SERVO_DIR = +1, если 180° — влево (тест: servo 180).
#define SERVO_US_MIN     500
#define SERVO_US_MAX     2400
#define SERVO_DIR        1

// TOF400F
#define TOF_MODE_SCAN    2       // 0 обычный, 1 точный, 2 дальний (до 4 м), 3 быстрый
#define TOF_OFFSET_MM    20      // датчик занижает на 2 см (проверено линейкой)
#define TOF_POS_X_MM     60      // ось сервы впереди центра машинки на N мм
#define TOF_MAX_MM       4000
#define TOF_PERIOD_MS    50      // период измерения модуля (с завода 500 мс — слишком медленно для скана)

// Скан
#define SCAN_STEP_DEG    3
#define SCAN_SETTLE_MS   45      // пауза после поворота сервы перед замером

// Ультразвук
#define US_MAX_MM        3000    // нет эха = «далеко»
#ifndef US_SLOT_MS
#define US_SLOT_MS       35      // мс между запусками ультразвуков (по очереди перед/зад)
#endif

// ИК пола: уровень на выходе, когда ПОД датчиком НЕТ пола (FC-51: HIGH)
#define CLIFF_ACTIVE     HIGH    // FC-51: LOW = видит пол, HIGH = пропасть (проверено на полу: 0)

// Защита
#define SAFE_FRONT_MM    180     // ближе — вперёд не едем (ручная езда)
#define SAFE_FRONT_PATH_MM 150   // на маршруте (путь проложен с запасом; 300 мм/с — нужен тормозной путь)
#define SAFE_REAR_MM     150     // ближе — назад не едем
#define CMD_TIMEOUT_MS   500     // ручное управление: нет команд — стоп
#define BAT_DIV_RATIO    (127.0f / 27.0f)
#define BAT_LOW_MV       9600    // ниже — моторы не включаются
#define BAT_PRESENT_MV   5000    // ниже — батарея выключена (питание по USB)

// Сеть
#define UDP_ROBOT_PORT   4211    // машинка слушает
#define UDP_PC_PORT      4210    // комп слушает
#define PC_TIMEOUT_MS    3000
#define HOSTNAME         "mapper"
