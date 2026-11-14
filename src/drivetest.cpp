// Тест ходовой: управление WASD/кнопками, прямой ШИМ (без энкодеров и регуляторов).
// Сборка: pio run -e drivetest -t upload   (или по Wi-Fi: pio run -e drivetest-ota -t upload)
// Страница: http://mapper.local  (или IP машинки) — в той же сети Wi-Fi, что и secrets.h
#ifdef DRIVE_TEST
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include "config.h"
#include "secrets.h"
#include "hw.h"

WebServer web(80);
Preferences prefs;

static bool invL = false, invR = false, swapLR = false;   // настраиваются со страницы, хранятся во flash
static float cmdL = 0, cmdR = 0;
static uint32_t cmdMs = 0;
static const uint32_t DEADMAN_MS = 400;   // нет команд — стоп

static void apply(float l, float r) {
  if (swapLR) { float t = l; l = r; r = t; }
  if (invL) l = -l;
  if (invR) r = -r;
  motorsSet(l * MOTOR_PWM_MAX, r * MOTOR_PWM_MAX);
}

static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>mapper: ходовая</title>
<style>
:root{--bg:#111;--card:#1c1c1f;--fg:#eee;--dim:#888;--acc:#4af;--ok:#3c6;--bad:#e44}
body{margin:0;background:var(--bg);color:var(--fg);font:16px system-ui,sans-serif;padding:12px;user-select:none;-webkit-user-select:none}
h1{font-size:18px;margin:0 0 10px}.card{background:var(--card);border-radius:12px;padding:12px;margin-bottom:10px}
.pad{display:grid;grid-template-columns:repeat(3,80px);grid-template-rows:repeat(2,80px);gap:8px;justify-content:center}
.k{background:#333;border-radius:12px;display:flex;align-items:center;justify-content:center;font-size:28px;font-weight:700;touch-action:none}
.k.on{background:var(--acc);color:#000}.row{display:flex;justify-content:space-between;font-variant-numeric:tabular-nums}
.bar{height:10px;background:#333;border-radius:5px;position:relative;overflow:hidden;margin:4px 0 10px}
.bar i{position:absolute;top:0;bottom:0;left:50%;background:var(--acc)}
label{display:block;margin:6px 0}input[type=range]{width:100%}.dim{color:var(--dim);font-size:13px}
</style></head><body>
<h1>mapper — ходовая <span class="dim">WASD / стрелки / кнопки · пробел — стоп</span> <span id="link" class="dim"></span></h1>
<div class="card"><div class="pad">
<div></div><div class="k" data-k="w">W</div><div></div>
<div class="k" data-k="a">A</div><div class="k" data-k="s">S</div><div class="k" data-k="d">D</div></div></div>
<div class="card"><div>Скорость: <b id="spv">70</b>%</div><input type="range" id="sp" min="20" max="100" value="70">
<div class="row"><span>Левая сторона</span><b id="lv">0</b></div><div class="bar"><i id="lb"></i></div>
<div class="row"><span>Правая сторона</span><b id="rv">0</b></div><div class="bar"><i id="rb"></i></div></div>
<div class="card"><b>Настройка направления</b>
<div class="dim">Поднимите колёса, зажмите W: все 4 колеса должны крутиться ВПЕРЁД. Если нет — исправьте галочками (запоминается).</div>
<label><input type="checkbox" id="invL"> Левая сторона крутится назад → перевернуть</label>
<label><input type="checkbox" id="invR"> Правая сторона крутится назад → перевернуть</label>
<label><input type="checkbox" id="swap"> На A поворачивает вправо → поменять стороны местами</label>
<button id="test" style="background:#333;color:#eee;border:0;border-radius:8px;padding:10px">Тест: только левая 1 с, потом только правая 1 с</button></div>
<div class="card dim" id="enc">энкодеры: —</div>
<script>
const $=id=>document.getElementById(id);const keys=new Set();
const map={w:'w',a:'a',s:'s',d:'d',arrowup:'w',arrowleft:'a',arrowdown:'s',arrowright:'d','ц':'w','ф':'a','ы':'s','в':'d'};
function mix(){const f=(keys.has('w')?1:0)-(keys.has('s')?1:0),t=(keys.has('a')?1:0)-(keys.has('d')?1:0);
 let l,r;if(f&&t){l=f*(t>0?0.35:1);r=f*(t>0?1:0.35)}else if(f){l=r=f}else{l=-t;r=t}
 const k=$('sp').value/100;return [l*k,r*k]}
function show(l,r){for(const [n,v] of [['l',l],['r',r]]){$(n+'v').textContent=Math.round(v*100);const b=$(n+'b');
 b.style.width=Math.abs(v)*50+'%';b.style.left=v<0?(50-Math.abs(v)*50)+'%':'50%'}
 document.querySelectorAll('.k').forEach(e=>e.classList.toggle('on',keys.has(e.dataset.k)))}
let fails=0;async function send(){const [l,r]=mix();show(l,r);
 try{const d=await (await fetch(`/drive?l=${l.toFixed(2)}&r=${r.toFixed(2)}&t=${Date.now()}`)).json();fails=0;
 $('link').textContent='● связь';$('link').style.color='#3c6';
 $('enc').textContent='энкодеры (фронтов с запуска) FL '+d.enc[0]+' · FR '+d.enc[1]+' · RL '+d.enc[2]+' · RR '+d.enc[3]+' · уровни '+d.pin.join('')}
 catch(e){if(++fails>2){$('link').textContent='● нет связи';$('link').style.color='#e44'}}}
setInterval(send,100);
addEventListener('keydown',e=>{const k=map[e.key.toLowerCase()];if(e.key==' '){keys.clear();e.preventDefault()}
 if(k){keys.add(k);e.preventDefault()}});
addEventListener('keyup',e=>{const k=map[e.key.toLowerCase()];if(k)keys.delete(k)});
addEventListener('blur',()=>keys.clear());
document.querySelectorAll('.k').forEach(el=>{const k=el.dataset.k;
 el.addEventListener('pointerdown',e=>{el.setPointerCapture(e.pointerId);keys.add(k)});
 for(const ev of ['pointerup','pointercancel','lostpointercapture'])el.addEventListener(ev,()=>keys.delete(k))});
$('sp').oninput=()=>$('spv').textContent=$('sp').value;
async function cfg(){await fetch(`/cfg?invL=${+$('invL').checked}&invR=${+$('invR').checked}&swap=${+$('swap').checked}`)}
for(const id of ['invL','invR','swap'])$(id).onchange=cfg;
$('test').onclick=()=>fetch('/test');
fetch('/cfg').then(r=>r.json()).then(c=>{$('invL').checked=c.invL;$('invR').checked=c.invR;$('swap').checked=c.swap});
</script></body></html>)HTML";

static void sendState() {
  char b[200];
  snprintf(b, sizeof(b), "{\"enc\":[%ld,%ld,%ld,%ld],\"pin\":[%d,%d,%d,%d]}", (long)encTotal[FL], (long)encTotal[FR],
           (long)encTotal[RL], (long)encTotal[RR], digitalRead(PIN_ENC_FL), digitalRead(PIN_ENC_FR),
           digitalRead(PIN_ENC_RL), digitalRead(PIN_ENC_RR));
  web.send(200, "application/json", b);
}

static uint32_t testStart = 0;
static float rawL = 0, rawR = 0;
static uint32_t rawUntil = 0;

void setup() {
  Serial.begin(115200);
  hwBegin();
  motorsStop();
  prefs.begin("drive", false);
  invL = prefs.getBool("invL", false);
  invR = prefs.getBool("invR", false);
  swapLR = prefs.getBool("swap", false);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  MDNS.begin(HOSTNAME);
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.begin();

  web.on("/", [] { web.sendHeader("Cache-Control", "no-store"); web.send_P(200, "text/html; charset=utf-8", PAGE); });
  web.on("/drive", [] {
    cmdL = constrain(web.arg("l").toFloat(), -1.f, 1.f);
    cmdR = constrain(web.arg("r").toFloat(), -1.f, 1.f);
    cmdMs = millis();
    sendState();
  });
  web.on("/cfg", [] {
    if (web.hasArg("invL")) {
      invL = web.arg("invL") == "1"; invR = web.arg("invR") == "1"; swapLR = web.arg("swap") == "1";
      prefs.putBool("invL", invL); prefs.putBool("invR", invR); prefs.putBool("swap", swapLR);
    }
    char b[80];
    snprintf(b, sizeof(b), "{\"invL\":%s,\"invR\":%s,\"swap\":%s}", invL ? "true" : "false", invR ? "true" : "false",
             swapLR ? "true" : "false");
    web.send(200, "application/json", b);
  });
  web.on("/enc", sendState);
  // подбор частоты ШИМ и сырой ШИМ без ограничения MOTOR_PWM_MAX (для калибровки)
  web.on("/pwm", [] {
    uint32_t f = web.arg("f").toInt();
    if (f >= 50 && f <= 20000)
      for (int p : {PIN_MOT_L_A, PIN_MOT_L_B, PIN_MOT_R_A, PIN_MOT_R_B}) ledcChangeFrequency(p, f, 10);
    web.send(200, "text/plain", String(f));
  });
  web.on("/raw", [] {
    rawL = web.arg("l").toFloat(); rawR = web.arg("r").toFloat();
    rawUntil = millis() + constrain((int)web.arg("ms").toInt(), 0, 3000);
    sendState();
  });   // только чтение (не трогает моторы)
  web.on("/test", [] { testStart = millis(); web.send(200, "text/plain", "ok"); });
  web.begin();
  MDNS.addService("http", "tcp", 80);
}

void loop() {
  hwUpdate();
  web.handleClient();
  ArduinoOTA.handle();
  uint32_t now = millis();

  static int32_t d[4];
  static uint32_t encMs = 0;
  if (now - encMs > 50) { encMs = now; encReadDeltas(d); }

  if (rawUntil && (int32_t)(rawUntil - now) > 0) {
    motorsSet(rawL, rawR);
  } else if (testStart && now - testStart < 2200) {
    rawUntil = 0;
    uint32_t t = now - testStart;
    if (t < 1000) apply(0.6f, 0);
    else if (t < 1200) apply(0, 0);
    else apply(0, 0.6f);
  } else {
    testStart = 0;
    if (now - cmdMs > DEADMAN_MS) cmdL = cmdR = 0;
    apply(cmdL, cmdR);
  }

  static uint32_t ledMs = 0;
  if (now - ledMs > 300) {
    ledMs = now;
    if (WiFi.status() != WL_CONNECTED) ledSet((now / 300) % 2 ? 40 : 0, 0, 0);
    else if (cmdL || cmdR) ledSet(0, 0, 40);
    else ledSet(0, 30, 0);
  }

  static uint32_t prMs = 0;
  if (now - prMs > 2000) {
    prMs = now;
    Serial.printf("IP %s  L=%.2f R=%.2f  enc %ld %ld %ld %ld\n", WiFi.localIP().toString().c_str(), cmdL, cmdR,
                  (long)encTotal[FL], (long)encTotal[FR], (long)encTotal[RL], (long)encTotal[RR]);
  }
}
#endif
