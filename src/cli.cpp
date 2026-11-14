// Консоль в мониторе порта: проверка железа по этапам и ручные команды.
#include "cli.h"
#include "config.h"
#include "hw.h"
#include "tof.h"
#include "robot.h"
#include "net.h"
#include <WiFi.h>

static char line[96];
static uint8_t len = 0;
static uint8_t watch = 0;          // 0 — выкл, 1 — статус, 2 — энкодеры
static uint32_t lastWatch = 0;

static void help() {
  Serial.println(F(
      "\n=== mapper: консоль ===\n"
      "st              — все датчики одной строкой\n"
      "w               — статус 5 раз/с (w ещё раз — выкл)\n"
      "enc             — счётчики энкодеров 5 раз/с (покрутите колёса рукой)\n"
      "mot L R [мс]    — сырой ШИМ сторон -1..1, напр. mot 0.5 0 1500 (КОЛЁСА В ВОЗДУХЕ)\n"
      "servo ГРАД      — серва 0..180 (90 = вперёд)\n"
      "calib           — калибровка гироскопа (не трогать машинку 2 с)\n"
      "fwd ММ          — проехать вперёд (назад — минус)\n"
      "turn ГРАД       — повернуться на угол (+ влево)\n"
      "scan            — скан 360°\n"
      "stop            — стоп\n"
      "zero            — обнулить положение (x=y=0, курс 0)\n"
      "wifi            — состояние сети\n"));
}

static void status() {
  const TofSample &t = tofLast();
  Serial.printf("mode=%s blk=%s | x=%.0f y=%.0f th=%.1f° | vL=%.0f vR=%.0f | US f=%u r=%u | cliff=%d | "
                "tof=%u%s | bat=%.2fV | imu=%s bias=%.2f | wifi=%s pc=%s\n",
                modeName(R.mode), R.block, R.x, R.y, R.th * RAD_TO_DEG, R.vL, R.vR, S.usFront, S.usRear,
                S.cliff, t.mm, tofOk() ? "" : "(нет связи)", S.batMv / 1000.0f, S.imuOk ? "ok" : "НЕТ",
                S.gyroBiasDps, WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "нет",
                netPcConnected() ? "да" : "нет");
}

static void exec(char *s) {
  char *cmd = strtok(s, " ");
  if (!cmd) return;
  char *a1 = strtok(nullptr, " "), *a2 = strtok(nullptr, " "), *a3 = strtok(nullptr, " ");
  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) help();
  else if (!strcmp(cmd, "st")) status();
  else if (!strcmp(cmd, "w")) watch = watch == 1 ? 0 : 1;
  else if (!strcmp(cmd, "enc")) { for (auto &e : encTotal) e = 0; watch = watch == 2 ? 0 : 2; }
  else if (!strcmp(cmd, "mot") && a1 && a2) robotTestMotors(atof(a1), atof(a2), a3 ? atoi(a3) : 1500);
  else if (!strcmp(cmd, "servo") && a1) servoDeg(atof(a1));
  else if (!strcmp(cmd, "calib")) { robotStop("calib"); Serial.println("калибровка..."); imuCalibrate(2000); Serial.printf("смещение %.3f °/с\n", S.gyroBiasDps); }
  else if (!strcmp(cmd, "fwd") && a1) {
    float d = atof(a1);
    float xy[2] = {R.x + d * cosf(R.th), R.y + d * sinf(R.th)};
    if (d < 0) Serial.println("назад по маршруту не ездим: машинка развернётся и поедет");
    robotPath(xy, 1, 1);
  }
  else if (!strcmp(cmd, "turn") && a1) robotTurnTo(R.th + atof(a1) * DEG_TO_RAD, 1);
  else if (!strcmp(cmd, "scan")) robotScan(1);
  else if (!strcmp(cmd, "stop")) robotStop("cli");
  else if (!strcmp(cmd, "zero")) robotSetPose(0, 0, 0);
  else if (!strcmp(cmd, "wifi")) Serial.printf("статус %d, IP %s, RSSI %d\n", WiFi.status(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
  else Serial.println("неизвестная команда, help — список");
}

void cliBegin() { help(); }

void cliUpdate() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = 0;
      if (len) exec(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
  if (watch && millis() - lastWatch > 200) {
    lastWatch = millis();
    if (watch == 1) status();
    else Serial.printf("FL=%ld FR=%ld RL=%ld RR=%ld  (фронтов; 1 оборот колеса = %d)\n", (long)encTotal[FL],
                       (long)encTotal[FR], (long)encTotal[RL], (long)encTotal[RR], ENC_SLOTS * ENC_EDGES);
  }
}
