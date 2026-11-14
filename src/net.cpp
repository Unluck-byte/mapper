#include "net.h"
#include "config.h"
#include "secrets.h"
#include "hw.h"
#include "tof.h"
#include "driver/gpio.h"
#include "robot.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

static WiFiUDP udp;
static IPAddress pcIp;
static bool pcStatic = false;
static uint32_t pcLastMs = 0;
static bool otaStarted = false;
static char buf[1600];

bool netPcConnected() { return pcStatic || (pcLastMs && millis() - pcLastMs < PC_TIMEOUT_MS); }

static void sendTo(const char *s, size_t n) {
  IPAddress dst = netPcConnected() ? pcIp : WiFi.broadcastIP();
  udp.beginPacket(dst, UDP_PC_PORT);
  udp.write((const uint8_t *)s, n);
  udp.endPacket();
}

void netBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  if (strlen(PC_IP) > 0 && pcIp.fromString(PC_IP)) pcStatic = true;
}

static void handleCmd(JsonDocument &d) {
  const char *c = d["c"] | "";
  if (!strcmp(c, "hb")) return;
  if (!strcmp(c, "drive")) { robotManual(d["v"] | 0.0f, d["w"] | 0.0f); return; }
  if (!strcmp(c, "stop"))  { robotStop("cmd"); return; }
  if (!strcmp(c, "scan"))  { robotScan(d["id"] | 0); return; }
  if (!strcmp(c, "turn"))  { robotTurnTo(d["th"] | 0.0f, d["id"] | 0); return; }
  if (!strcmp(c, "pose"))  { robotSetPose(d["x"] | 0.0f, d["y"] | 0.0f, d["th"] | 0.0f); return; }
  if (!strcmp(c, "mot"))   { robotTestMotors(d["l"] | 0.0f, d["r"] | 0.0f, d["ms"] | 1000); return; }   // тест/калибровка
  if (!strcmp(c, "tofmode")) { if (R.mode == M_IDLE) tofBegin(d["m"] | TOF_MODE_SCAN); return; }   // отладка режимов TOF
  if (!strcmp(c, "tofreg") && R.mode == M_IDLE) {      // отладка: {"c":"tofreg","r":0,"n":16[,"w":val]}
    uint16_t r0 = d["r"] | 0, n = d["n"] | 1;
    if (!d["w"].isNull()) tofWriteReg(r0, d["w"].as<uint16_t>());
    char o[600]; int k = snprintf(o, sizeof(o), "{\"t\":\"tofreg\",\"r\":%u,\"v\":[", r0);
    for (uint16_t i = 0; i < n && i < 48 && k < 560; i++) {
      uint16_t v; bool ok = tofReadReg(r0 + i, v);
      k += snprintf(o + k, sizeof(o) - k, "%s%d", i ? "," : "", ok ? (int)v : -1);
    }
    k += snprintf(o + k, sizeof(o) - k, "]}");
    udp.beginPacket(udp.remoteIP(), udp.remotePort()); udp.write((const uint8_t *)o, k); udp.endPacket();
    return;
  }
  if (!strcmp(c, "servo")) { if (R.mode == M_IDLE) servoDeg(d["deg"] | 90.0f); return; }
  if (!strcmp(c, "path")) {
    JsonArray pts = d["pts"].as<JsonArray>();
    static float xy[96];
    int n = 0;
    for (JsonArray p : pts) {
      if (n >= 48) break;
      xy[2 * n] = p[0] | 0.0f;
      xy[2 * n + 1] = p[1] | 0.0f;
      n++;
    }
    if (!robotPath(xy, n, d["id"] | 0)) {
      int len = snprintf(buf, sizeof(buf), "{\"t\":\"ev\",\"ev\":\"error\",\"msg\":\"bad path\"}");
      sendTo(buf, len);
    }
    return;
  }
}

static void sendTelemetry() {
  const TofSample &t = tofLast();
  int len = snprintf(buf, sizeof(buf),
      "{\"t\":\"tel\",\"x\":%.1f,\"y\":%.1f,\"th\":%.4f,\"vl\":%.0f,\"vr\":%.0f,\"v\":%.0f,\"w\":%.2f,"
      "\"mode\":\"%s\",\"blk\":\"%s\",\"wp\":%d,\"wpn\":%d,\"id\":%lu,"
      "\"usf\":%u,\"usr\":%u,\"cliff\":%d,\"tof\":%u,\"servo\":%.0f,"
      "\"bat\":%u,\"imu\":%d,\"tofok\":%d,\"rssi\":%d,\"up\":%lu,"
      "\"enc\":[%ld,%ld,%ld,%ld],\"encpin\":[%d,%d,%d,%d],\"gz\":%.2f,\"cliffraw\":%d}",
      R.x, R.y, R.th, R.vL, R.vR, R.v, R.w, modeName(R.mode), R.block, R.wpIdx, R.wpCount,
      (unsigned long)R.taskId, S.usFront, S.usRear, S.cliff ? 1 : 0, t.mm, servoGetDeg(), S.batMv,
      S.imuOk ? 1 : 0, tofOk() ? 1 : 0, WiFi.RSSI(), (unsigned long)(millis() / 1000),
      (long)encTotal[FL], (long)encTotal[FR], (long)encTotal[RL], (long)encTotal[RR], gpio_get_level((gpio_num_t)PIN_ENC_FL), gpio_get_level((gpio_num_t)PIN_ENC_FR),
      gpio_get_level((gpio_num_t)PIN_ENC_RL), gpio_get_level((gpio_num_t)PIN_ENC_RR), S.yawRate * RAD_TO_DEG, digitalRead(PIN_CLIFF));
  sendTo(buf, len);
}

static void sendScan() {
  int n;
  float x, y;
  const ScanPoint *p = robotScanPoints(n, x, y);
  const int CH = 40;
  int parts = (n + CH - 1) / CH;
  for (int k = 0; k < parts; k++) {
    int len = snprintf(buf, sizeof(buf), "{\"t\":\"scan\",\"id\":%lu,\"part\":%d,\"parts\":%d,\"x\":%.1f,\"y\":%.1f,\"pts\":[",
                       (unsigned long)R.taskId, k, parts, x, y);
    for (int i = k * CH; i < min(n, (k + 1) * CH); i++)
      len += snprintf(buf + len, sizeof(buf) - len, "%s[%d,%d,%u]", i == k * CH ? "" : ",", p[i].thMrad, p[i].relDeci, p[i].mm);
    len += snprintf(buf + len, sizeof(buf) - len, "]}");
    sendTo(buf, len);
    delay(2);
  }
}

void netUpdate() {
  static uint32_t lastTel = 0;
  static bool wasConnected = false;
  bool wifi = WiFi.status() == WL_CONNECTED;
  if (wifi && !wasConnected) {
    udp.begin(UDP_ROBOT_PORT);
    Serial.printf("Wi-Fi: %s  IP %s\n", WIFI_SSID, WiFi.localIP().toString().c_str());
    if (!otaStarted) {
      ArduinoOTA.setHostname(HOSTNAME);
      ArduinoOTA.onStart([]() { motorsStop(); robotStop("ota"); });
      ArduinoOTA.begin();
      otaStarted = true;
    }
  }
  wasConnected = wifi;
  if (!wifi) return;
  ArduinoOTA.handle();

  // входящие команды
  int sz;
  while ((sz = udp.parsePacket()) > 0) {
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = 0;
    if (!pcStatic) pcIp = udp.remoteIP();
    pcLastMs = millis();
    JsonDocument d;
    if (deserializeJson(d, buf) == DeserializationError::Ok) handleCmd(d);
  }

  // связь с компом потеряна — не ездим вслепую
  if (!netPcConnected() && (R.mode == M_MANUAL || R.mode == M_PATH || R.mode == M_TURN)) robotStop("link");

  // события
  char ev[160];
  while (robotPopEvent(ev, sizeof(ev))) {
    sendTo(ev, strlen(ev));
    Serial.println(ev);
    if (strstr(ev, "\"scan_done\"")) sendScan();
  }

  if (millis() - lastTel >= 100) {
    lastTel = millis();
    sendTelemetry();
  }
}
