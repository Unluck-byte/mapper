#include "robot.h"
#include "config.h"
#include "hw.h"
#include "tof.h"

RobotState R;

// скорость машинки при ШИМ = MOTOR_PWM_MAX (для прямой подачи; уточняется регулятором)
#define SPEED_AT_PWM_MIN 260.0f   // мм/с на полу при MOTOR_PWM_MIN (замер 9 В)
#define SPEED_AT_PWM_MAX 1000.0f  // мм/с на полу при MOTOR_PWM_MAX (замер: 75% ≈ 1000)
#define KP_SPEED  0.0010f
#define KI_SPEED  0.0025f

static float thOffset = 0;          // th = S.yaw + thOffset
static uint32_t lastCtl = 0;
static float dirL = 1, dirR = 1;    // направление вращения сторон (энкодеры его не знают)
static float vCmd = 0, wCmd = 0;    // после плавного разгона
static float iL = 0, iR = 0;        // интеграторы
static uint32_t stallMsL = 0, stallMsR = 0;
static uint32_t stillSince = 0;

// ручное управление
static float manV = 0, manW = 0;
static uint32_t manMs = 0;

// маршрут
#define MAX_WP 48
static float wpX[MAX_WP], wpY[MAX_WP];
static bool rotating = true;
static float turnTarget = 0;

// тест моторов
static float testL = 0, testR = 0;
static uint32_t testUntil = 0;

// скан
enum ScanPhase { SC_SETTLE, SC_SWEEP, SC_ROTATE, SC_DONE };
static ScanPhase scPh;
static int scHalf, scAngle;
static uint32_t scT;
static float scTarget;
#define MAX_SCAN 256
static ScanPoint scPts[MAX_SCAN];
static int scN = 0;
static float scX = 0, scY = 0;

// события
#define EVQ 8
static char evq[EVQ][160];
static int evHead = 0, evTail = 0;

static void emit(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(evq[evHead], sizeof(evq[0]), fmt, ap);
  va_end(ap);
  evHead = (evHead + 1) % EVQ;
  if (evHead == evTail) evTail = (evTail + 1) % EVQ;
}

bool robotPopEvent(char *buf, size_t len) {
  if (evHead == evTail) return false;
  strlcpy(buf, evq[evTail], len);
  evTail = (evTail + 1) % EVQ;
  return true;
}

const char *modeName(Mode m) {
  switch (m) {
    case M_IDLE: return "idle";
    case M_MANUAL: return "manual";
    case M_PATH: return "path";
    case M_TURN: return "turn";
    case M_SCAN: return "scan";
    case M_TEST: return "test";
  }
  return "?";
}

static float wrapPi(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

// ------------------------------------------------------------------ команды
void robotBegin() {
  lastCtl = millis();
  thOffset = -S.yaw;
}

void robotManual(float v, float w) {
  if (R.mode == M_SCAN) return;              // скан не прерываем джойстиком
  manV = constrain(v, -MAX_SPEED_MM_S, MAX_SPEED_MM_S);
  manW = constrain(w, -TURN_RATE_RAD_S, TURN_RATE_RAD_S);
  manMs = millis();
  R.mode = M_MANUAL;
}

void robotStop(const char *why) {
  Mode was = R.mode;
  R.mode = M_IDLE;
  vCmd = wCmd = 0;
  motorsStop();
  if (was == M_SCAN) servoDeg(90);
  if (was == M_PATH || was == M_TURN || was == M_SCAN)
    emit("{\"t\":\"ev\",\"ev\":\"stopped\",\"id\":%lu,\"why\":\"%s\"}", (unsigned long)R.taskId, why);
  // скан прервали (упор на развороте и т.п.) — отдаём то, что успели снять, а не выбрасываем
  if (was == M_SCAN && scN >= 20)
    emit("{\"t\":\"ev\",\"ev\":\"scan_done\",\"id\":%lu,\"n\":%d,\"partial\":1}", (unsigned long)R.taskId, scN);
}

bool robotPath(const float *xy, int n, uint32_t id) {
  if (n < 1 || n > MAX_WP) return false;
  for (int i = 0; i < n; i++) { wpX[i] = xy[2 * i]; wpY[i] = xy[2 * i + 1]; }
  R.wpCount = n;
  R.wpIdx = 0;
  R.taskId = id;
  rotating = true;
  R.mode = M_PATH;
  return true;
}

void robotTurnTo(float th, uint32_t id) {
  turnTarget = wrapPi(th);
  R.taskId = id;
  R.mode = M_TURN;
}

void robotScan(uint32_t id) {
  R.taskId = id;
  R.mode = M_SCAN;
  scPh = SC_SETTLE;
  scHalf = 0;
  scN = 0;
  scT = millis();
  servoDeg(0);
}

void robotSetPose(float x, float y, float th) {
  R.x = x;
  R.y = y;
  R.th = wrapPi(th);
  thOffset = R.th - S.yaw;
}

void robotTestMotors(float l, float r, uint32_t ms) {
  testL = l;
  testR = r;
  testUntil = millis() + ms;
  R.mode = M_TEST;
}

const ScanPoint *robotScanPoints(int &n, float &x, float &y) {
  n = scN;
  x = scX;
  y = scY;
  return scPts;
}

// ------------------------------------------------------------------ вспомогательное
// путь одной стороны по двум энкодерам: пробуксовка/битый энкодер отбрасываются
static float sideEdges(int32_t a, int32_t b) {
  if (a == 0 && b == 0) return 0;
  if (a == 0 && b >= 3) return b;
  if (b == 0 && a >= 3) return a;
  int32_t lo = min(a, b), hi = max(a, b);
  if (lo > 0 && hi > lo * 1.6f) return lo;     // одно колесо буксует
  return (a + b) * 0.5f;
}

static float sideCtl(float target, float meas, float &integ, uint32_t &stallMs, float dt) {
  if (fabsf(target) < 5) { integ = 0; stallMs = 0; return 0; }
  float s = target > 0 ? 1 : -1;
  float ff = MOTOR_PWM_MIN + max(0.0f, fabsf(target) - SPEED_AT_PWM_MIN) / (SPEED_AT_PWM_MAX - SPEED_AT_PWM_MIN) *
                                 (MOTOR_PWM_MAX - MOTOR_PWM_MIN);
  float err = fabsf(target) - fabsf(meas);
  float ni = constrain(integ + KI_SPEED * err * dt, -0.3f, 0.3f);
  float raw = ff + KP_SPEED * err + ni;
  // интеграл не «накручиваем», когда упёрлись в предел (цель медленнее порога трогания и т.п.)
  if (!((raw < MOTOR_PWM_MIN && err < 0) || (raw > MOTOR_PWM_MAX && err > 0))) integ = ni;
  // ниже порога трогания не опускаемся: колесо либо крутится, либо выключено
  float u = constrain(ff + KP_SPEED * err + integ, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
  // упор: ШИМ большой, а колёса стоят
  if (u > MOTOR_PWM_MIN + 0.15f && fabsf(meas) < 10) {
    if (!stallMs) stallMs = millis();
  } else stallMs = 0;
  return s * u;
}

static bool frontBlocked() {
  uint16_t lim = R.mode == M_PATH ? SAFE_FRONT_PATH_MM : SAFE_FRONT_MM;
  bool us = S.usFront > 0 && S.usFront < lim;
  const TofSample &t = tofLast();
  bool tof = fabsf(servoGetDeg() - 90) < 5 && t.mm > 0 && millis() - t.ms < 200 &&
             t.mm < lim && R.mode != M_SCAN;
  return us || tof;
}

// ------------------------------------------------------------------ скан
static void scanStep(uint32_t now, float &vt, float &wt) {
  vt = 0;
  wt = 0;
  switch (scPh) {
    case SC_SETTLE:
      imuSetStationary(true);
      if (now - scT > 500) {
        scPh = SC_SWEEP;
        scAngle = scHalf == 0 ? 0 : 180;
        servoDeg(scAngle);
        if (scHalf == 0) { scX = R.x; scY = R.y; }
      }
      break;
    case SC_SWEEP: {
      const TofSample &t = tofLast();
      // замер должен быть сделан целиком ПОСЛЕ того, как серва встала: ответ пришёл позже, чем
      // остановка + период измерения модуля (иначе это старое расстояние с прошлого угла)
      uint32_t ready = servoMovedMs() + SCAN_SETTLE_MS;
      if (now < ready || t.ms < ready + TOF_PERIOD_MS + 15) break;
      if (scN < MAX_SCAN) {
        ScanPoint &p = scPts[scN++];
        p.thMrad = (int16_t)(R.th * 1000);
        p.relDeci = (int16_t)((scAngle - 90) * SERVO_DIR * 10);
        p.mm = t.mm;
      }
      int next = scAngle + (scHalf == 0 ? SCAN_STEP_DEG : -SCAN_STEP_DEG);
      if (next >= 0 && next <= 180) {
        scAngle = next;
        servoDeg(scAngle);
      } else if (scHalf == 0) {
        scHalf = 1;
        scTarget = wrapPi(R.th + PI);
        scPh = SC_ROTATE;
        imuSetStationary(false);
      } else {
        scPh = SC_DONE;
      }
      break;
    }
    case SC_ROTATE: {
      float err = wrapPi(scTarget - R.th);
      wt = constrain(2.5f * err, -TURN_RATE_RAD_S, TURN_RATE_RAD_S);
      if (fabsf(err) < 0.05f) { wt = 0; scPh = SC_SETTLE; scT = now; }
      break;
    }
    case SC_DONE:
      servoDeg(90);
      R.mode = M_IDLE;
      emit("{\"t\":\"ev\",\"ev\":\"scan_done\",\"id\":%lu,\"n\":%d}", (unsigned long)R.taskId, scN);
      break;
  }
}

// ------------------------------------------------------------------ главный цикл 50 Гц
void robotUpdate() {
  uint32_t now = millis();
  if (now - lastCtl < 20) return;
  float dt = (now - lastCtl) * 1e-3f;
  lastCtl = now;

  // --- одометрия
  int32_t e[4];
  encReadDeltas(e);
  if (motorLeft() != 0) dirL = motorLeft() > 0 ? 1 : -1;
  if (motorRight() != 0) dirR = motorRight() > 0 ? 1 : -1;
  float dl = sideEdges(e[FL], e[RL]) * MM_PER_EDGE * dirL;
  float dr = sideEdges(e[FR], e[RR]) * MM_PER_EDGE * dirR;
  R.vL += 0.3f * (dl / dt - R.vL);
  R.vR += 0.3f * (dr / dt - R.vR);
  float ds = (dl + dr) * 0.5f;
  float thNew = S.imuOk ? wrapPi(S.yaw + thOffset) : wrapPi(R.th + (dr - dl) / TRACK_MM);
  float thMid = R.th + wrapPi(thNew - R.th) * 0.5f;
  R.x += ds * cosf(thMid);
  R.y += ds * sinf(thMid);
  R.th = thNew;

  // --- тест моторов: сырой ШИМ
  if (R.mode == M_TEST) {
    if (now > testUntil) { R.mode = M_IDLE; motorsStop(); }
    else motorsSet(testL, testR);
    return;
  }

  // --- цель от режима
  float vt = 0, wt = 0;
  switch (R.mode) {
    case M_IDLE:
      break;
    case M_MANUAL:
      if (now - manMs < CMD_TIMEOUT_MS) { vt = manV; wt = manW; }
      else R.mode = M_IDLE;
      break;
    case M_TURN: {
      float err = wrapPi(turnTarget - R.th);
      wt = constrain(2.5f * err, -TURN_RATE_RAD_S, TURN_RATE_RAD_S);
      if (fabsf(err) < 0.04f) {
        R.mode = M_IDLE;
        emit("{\"t\":\"ev\",\"ev\":\"turn_done\",\"id\":%lu}", (unsigned long)R.taskId);
      }
      break;
    }
    case M_PATH: {
      float dx = wpX[R.wpIdx] - R.x, dy = wpY[R.wpIdx] - R.y;
      float dist = sqrtf(dx * dx + dy * dy);
      bool last = R.wpIdx == R.wpCount - 1;
      if (dist < (last ? 40 : 70)) {
        if (++R.wpIdx >= R.wpCount) {
          R.mode = M_IDLE;
          emit("{\"t\":\"ev\",\"ev\":\"path_done\",\"id\":%lu}", (unsigned long)R.taskId);
          break;
        }
        rotating = true;
        break;
      }
      float err = wrapPi(atan2f(dy, dx) - R.th);
      if (rotating) {
        wt = constrain(2.5f * err, -TURN_RATE_RAD_S, TURN_RATE_RAD_S);
        if (fabsf(err) < 0.07f) rotating = false;
      } else {
        if (fabsf(err) > 0.5f) { rotating = true; break; }
        vt = min(PATH_SPEED_MM_S, 1.5f * dist + 50);
        wt = constrain(2.0f * err, -0.8f, 0.8f);
      }
      break;
    }
    case M_SCAN:
      scanStep(now, vt, wt);
      break;
    default:
      break;
  }

  // --- защита
  const char *blk = "";
  bool batLow = S.batMv && S.batMv < BAT_LOW_MV;
  if (batLow) { vt = 0; wt = 0; blk = "bat"; }
  if (vt > 0 && S.cliff) { vt = 0; blk = "cliff"; }
  else if (vt > 0 && frontBlocked()) { vt = 0; blk = "front"; }
  if (vt < 0 && S.usRear > 0 && S.usRear < SAFE_REAR_MM) { vt = 0; blk = "rear"; }
  if (*blk && R.mode == M_PATH) robotStop(blk);
  if (*blk && strcmp(blk, R.block) != 0)
    emit("{\"t\":\"ev\",\"ev\":\"blocked\",\"why\":\"%s\"}", blk);
  R.block = blk;
  if (R.mode == M_IDLE) { vt = 0; wt = 0; }

  // --- плавный разгон (торможение по защите — сразу)
  float dv = ACCEL_MM_S2 * dt;
  if (vt == 0 && *blk) vCmd = 0;
  else vCmd += constrain(vt - vCmd, -2 * dv, dv);
  wCmd += constrain(wt - wCmd, -20 * dt, 20 * dt);
  R.v = vCmd;
  R.w = wCmd;

  // --- гироскоп «стоим»
  bool still = fabsf(vCmd) < 1 && fabsf(wCmd) < 0.01f && fabsf(R.vL) < 5 && fabsf(R.vR) < 5;
  if (!still) stillSince = 0;
  else if (!stillSince) stillSince = now;
  if (R.mode != M_SCAN) imuSetStationary(still && now - stillSince > 500);

  // --- регуляторы скоростей сторон
  float tL = vCmd - wCmd * TRACK_MM * 0.5f;
  float tR = vCmd + wCmd * TRACK_MM * 0.5f;
  float uL = sideCtl(tL, R.vL, iL, stallMsL, dt);
  float uR = sideCtl(tR, R.vR, iR, stallMsR, dt);
  if ((stallMsL && now - stallMsL > 1500) || (stallMsR && now - stallMsR > 1500)) {
    stallMsL = stallMsR = 0;
    R.block = "stall";
    emit("{\"t\":\"ev\",\"ev\":\"blocked\",\"why\":\"stall\"}");
    robotStop("stall");
    uL = uR = 0;
  }
  motorsSet(uL, uR);
}
