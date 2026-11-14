"""Сервер mapper: UDP-связь с машинкой, карта, ICP, A*, веб-интерфейс.

Запуск:  pc/.venv/bin/python -m mapper.server      (из папки pc/)
Открыть: http://localhost:8080  (с телефона — http://<ip-компа>:8080)
"""
import asyncio
import json
import math
import os
import socket
import time
from pathlib import Path

import numpy as np
from aiohttp import web, WSMsgType

from . import config as C
from .grid import Grid
from .icp import match, edge_filter, beam_correct
from .planner import plan, ahead_blocked

WEB_DIR = Path(__file__).resolve().parent.parent / "web"
MAP_DIR = Path(__file__).resolve().parent.parent / "maps"
SCAN_DIR = Path(__file__).resolve().parent.parent / "scans"


class App:
    def __init__(self):
        self.grid = Grid()
        self.tel = {}
        self.robot_addr = None
        self.last_tel = 0.0
        self.browsers = set()
        self.scan_parts = {}
        self.last_scan_pts = []
        self.path = []
        self.task_id = int(time.time()) % 100000
        self.transport = None
        self.sent_map_version = -1
        self.goal = None
        self.retries = 0
        self.replan_after_scan = False
        self.auto_scan = True
        self.travel_mm = 0.0
        self.travel_rad = 0.0

    # ------------------------------------------------------------------ машинка
    def send_robot(self, msg):
        if self.transport and self.robot_addr:
            self.transport.sendto(json.dumps(msg).encode(), (self.robot_addr[0], C.ROBOT_PORT))

    def on_udp(self, data, addr):
        try:
            m = json.loads(data.decode())
        except Exception:
            return
        t = m.get("t")
        if t == "tel":
            if self.robot_addr is None or self.robot_addr[0] != addr[0]:
                self.log(f"машинка найдена: {addr[0]}")
            self.robot_addr = addr
            if self.tel:
                self.travel_mm += math.hypot(m["x"] - self.tel["x"], m["y"] - self.tel["y"])
                self.travel_rad += abs((m["th"] - self.tel["th"] + math.pi) % (2 * math.pi) - math.pi)
            self.tel = m
            self.last_tel = time.time()
            if m.get("mode") != "scan":
                self.grid.clear_trail(m["x"], m["y"])
            self.broadcast({"t": "tel", **m, "link": True})
        elif t == "ev":
            self.on_event(m)
        elif t == "scan":
            self.on_scan_part(m)

    def on_event(self, m):
        ev = m.get("ev")
        names = {"scan_done": "скан готов", "path_done": "приехали", "turn_done": "поворот готов",
                 "stopped": "остановка", "blocked": "защита", "error": "ошибка"}
        why = m.get("why", "")
        self.log(f"{names.get(ev, ev)}{': ' + why if why else ''}")
        if ev in ("path_done", "stopped"):
            self.path = []
            self.broadcast({"t": "path", "pts": []})
        if ev == "path_done":
            self.goal = None
            if self.auto_scan:
                self.send_robot({"c": "scan", "id": self.next_id()})
        if ev == "stopped" and why in ("front", "cliff", "stall") and self.goal:
            if self.retries < C.REPLAN_RETRIES:
                self.retries += 1
                self.replan_after_scan = True
                self.log(f"препятствие — сканирую и строю новый маршрут ({self.retries}/{C.REPLAN_RETRIES})")
                self.send_robot({"c": "scan", "id": self.next_id()})
            else:
                self.log("не получилось доехать: путь всё время перекрыт")
                self.goal = None
        self.broadcast({"t": "ev", **m})

    def on_scan_part(self, m):
        key = m["id"]
        parts = self.scan_parts.setdefault(key, {})
        parts[m["part"]] = m
        if len(parts) == m["parts"]:
            del self.scan_parts[key]
            pts = []
            for k in sorted(parts):
                pts += parts[k]["pts"]
            self.process_scan(m["x"], m["y"], pts)

    # ------------------------------------------------------------------ скан -> карта
    def process_scan(self, x, y, raw):
        try:                       # сырой скан — для отладки датчика
            SCAN_DIR.mkdir(exist_ok=True)
            (SCAN_DIR / time.strftime("scan_%Y%m%d_%H%M%S.json")).write_text(json.dumps({"x": x, "y": y, "pts": raw}))
        except OSError:
            pass
        rays, hits = [], []
        for th, rel, mm in beam_correct(raw):
            sx = x + C.TOF_POS_X_MM * math.cos(th)
            sy = y + C.TOF_POS_X_MM * math.sin(th)
            a = th + rel
            if mm and mm <= C.TOF_MAX_MM:
                ex, ey = sx + mm * math.cos(a), sy + mm * math.sin(a)
                rays.append([sx, sy, ex, ey, True])
                hits.append((ex, ey, mm))
            else:
                d = C.NO_HIT_FREE_MM
                rays.append([sx, sy, sx + d * math.cos(a), sy + d * math.sin(a), False])

        corr = None
        hits_d = hits
        hits = np.array([h[:2] for h in hits]) if hits else np.zeros((0, 2))
        if (self.grid.L > C.OCC_T).sum() >= C.ICP_MIN_MAP_CELLS and len(hits) >= 15:
            pts = edge_filter(hits_d) or [h[:2] for h in hits_d]
            res = match(self.grid.L, np.array(pts), x, y)
            if res:
                gain = res["score"] - res["base"]
                shift = math.hypot(res["dx"], res["dy"])
                m_ = self.travel_mm / 1000.0
                max_shift = C.ODOM_ERR_BASE_MM + C.ODOM_ERR_PER_M * self.travel_mm
                max_rot = math.radians(C.ODOM_ROT_BASE_DEG + C.ODOM_ROT_PER_M * (m_ + self.travel_rad / (2 * math.pi)))
                plausible = shift <= max_shift and abs(res["dth"]) <= max_rot
                if shift > C.MATCH_BIG_SHIFT_MM and res["score"] < C.MATCH_BIG_MIN_SCORE:
                    plausible = False      # большая поправка только при уверенном совпадении
                if res["score"] >= C.MATCH_MIN_SCORE and (gain >= C.MATCH_MIN_GAIN or shift < 15) and plausible:
                    corr = res
                else:
                    why = "неправдоподобно большая" if not plausible else f"качество {res['score']:.2f} (было {res['base']:.2f})"
                    self.log(f"совмещение отклонено: {why} — {shift:.0f} мм, {math.degrees(res['dth']):+.1f}°")

        if corr:
            dth, shift = corr["dth"], math.hypot(corr["dx"], corr["dy"])
            c, s_ = math.cos(dth), math.sin(dth)
            R = np.array([[c, -s_], [s_, c]])
            o = np.array([x, y])
            t = o + np.array([corr["dx"], corr["dy"]]) - R @ o       # поворот вокруг машинки + сдвиг
            err = corr["score"]
            for r in rays:
                s1 = R @ np.array(r[0:2]) + t
                e1 = R @ np.array(r[2:4]) + t
                r[0], r[1], r[2], r[3] = s1[0], s1[1], e1[0], e1[1]
            hits = hits @ R.T + t if len(hits) else hits
            # исправить положение машинки (она стоит после скана)
            if self.tel:
                rx, ry, rth = self.tel["x"], self.tel["y"], self.tel["th"]
                p = R @ np.array([rx, ry]) + t
                self.send_robot({"c": "pose", "x": round(float(p[0]), 1), "y": round(float(p[1]), 1),
                                 "th": round(rth + dth, 4)})
                if "gt" in self.tel:   # только симулятор: истинная ошибка до/после
                    g = self.tel["gt"]
                    e0 = math.hypot(rx - g[0], ry - g[1]), math.degrees(abs((rth - g[2] + math.pi) % (2 * math.pi) - math.pi))
                    e1 = math.hypot(p[0] - g[0], p[1] - g[1]), math.degrees(abs((rth + dth - g[2] + math.pi) % (2 * math.pi) - math.pi))
                    # что было бы, если исправлять только поворот
                    pr = R @ np.array([rx - x, ry - y]) + np.array([x, y])
                    e2 = math.hypot(pr[0] - g[0], pr[1] - g[1])
                    self.log(f"[sim] ошибка до {e0[0]:.0f} мм/{e0[1]:.1f}° → после {e1[0]:.0f} мм/{e1[1]:.1f}° | только поворот: {e2:.0f} мм | "
                             f"score {corr['score']:.3f} base {corr['base']:.3f} gain {corr['score']-corr['base']:.3f} shift {shift:.0f} weak {corr['weak']}")
            self.log(f"положение уточнено: {shift:.0f} мм, {math.degrees(dth):+.1f}° (качество {err:.2f})")

        self.grid.integrate(rays)
        self.travel_mm = 0.0
        self.travel_rad = 0.0
        self.last_scan_pts = [[round(float(a), 1), round(float(b), 1)] for a, b in hits]
        self.broadcast({"t": "scanpts", "pts": self.last_scan_pts})
        self.push_map()
        self.log(f"скан: {len(raw)} лучей, {len(hits)} попаданий")
        if self.replan_after_scan and self.goal:
            self.replan_after_scan = False
            asyncio.ensure_future(self.do_goto(self.goal))

    # ------------------------------------------------------------------ браузеры
    def log(self, msg):
        print(time.strftime("%H:%M:%S"), msg, flush=True)
        self.broadcast({"t": "log", "msg": msg})

    def broadcast(self, msg):
        if not self.browsers:
            return
        s = json.dumps(msg)
        for ws in list(self.browsers):
            if not ws.closed:
                asyncio.ensure_future(self._send_ws(ws, s))

    async def _send_ws(self, ws, s):
        try:
            await ws.send_str(s)
        except Exception:
            self.browsers.discard(ws)

    def cfg_msg(self):
        return {"t": "cfg", "auto_scan": self.auto_scan, "noise": self.grid.noise_level, "noise_max": C.NOISE_LEVEL_MAX}

    def map_msg(self):
        return {"t": "map", "size": C.SIZE, "res": C.RES_MM, "data": self.grid.encoded()}

    def push_map(self):
        if self.grid.version != self.sent_map_version:
            self.sent_map_version = self.grid.version
            self.broadcast(self.map_msg())

    async def do_goto(self, goal):
        if not self.tel:
            self.log("нет связи с машинкой")
            return
        start = (self.tel["x"], self.tel["y"])
        t0 = time.time()
        pts, msg = await asyncio.get_running_loop().run_in_executor(None, plan, self.grid, start, goal)
        if pts is None:
            self.log(f"маршрут не найден: {msg}")
            self.goal = None
            return
        self.path = [list(start)] + pts
        self.send_robot({"c": "path", "id": self.next_id(), "pts": pts})
        self.broadcast({"t": "path", "pts": self.path})
        self.log(f"маршрут: {msg}, {(time.time()-t0)*1000:.0f} мс")

    def next_id(self):
        self.task_id += 1
        return self.task_id

    async def on_browser(self, m, ws):
        c = m.get("c")
        if c == "drive":
            v = max(-C.MAX_V, min(C.MAX_V, float(m.get("v", 0))))
            w = max(-C.MAX_W, min(C.MAX_W, float(m.get("w", 0))))
            # не въезжать в известную стену даже вручную
            if v > 0 and self.tel and ahead_blocked(self.grid, self.tel["x"], self.tel["y"], self.tel["th"], 100 + 0.5 * v):
                v = 0.0
            self.send_robot({"c": "drive", "v": round(v, 1), "w": round(w, 3)})
        elif c == "stop":
            self.goal = None
            self.replan_after_scan = False
            self.send_robot({"c": "stop"})
            self.path = []
            self.broadcast({"t": "path", "pts": []})
        elif c == "scan":
            self.send_robot({"c": "scan", "id": self.next_id()})
            self.log("скан запущен")
        elif c == "goto":
            self.goal = (float(m["x"]), float(m["y"]))
            self.retries = 0
            self.replan_after_scan = False
            await self.do_goto(self.goal)
        elif c == "auto_scan":
            self.auto_scan = bool(m.get("on"))
            self.log("автоскан после поездки: " + ("вкл" if self.auto_scan else "выкл"))
            self.broadcast(self.cfg_msg())
        elif c == "noise":
            self.grid.noise_level = max(0, min(C.NOISE_LEVEL_MAX, int(m.get("level", 0))))
            self.grid.version += 1
            self.push_map()
            self.broadcast(self.cfg_msg())
        elif c == "clear_map":
            self.grid.clear()
            self.last_scan_pts = []
            self.broadcast({"t": "scanpts", "pts": []})
            self.push_map()
            self.log("карта очищена")
        elif c == "reset_pose":
            self.send_robot({"c": "pose", "x": 0, "y": 0, "th": 0})
            self.log("положение машинки обнулено")
        elif c == "save_map":
            MAP_DIR.mkdir(exist_ok=True)
            name = time.strftime("map_%Y%m%d_%H%M%S.npy")
            self.grid.save(MAP_DIR / name)
            self.log(f"карта сохранена: pc/maps/{name}")
        elif c == "load_map":
            files = sorted(MAP_DIR.glob("map_*.npy")) if MAP_DIR.exists() else []
            if not files:
                self.log("сохранённых карт нет")
                return
            self.grid.load(files[-1])
            self.push_map()
            self.log(f"загружена карта {files[-1].name}. Поставьте машинку туда же, где начинали, и нажмите «Скан»")

    # ------------------------------------------------------------------ фоновые задачи
    async def heartbeat(self):
        n = 0
        while True:
            n += 1
            if n % 4 == 0:
                self.push_map()          # след машинки очищает карту — отправлять не чаще раза в 2 с
            self.send_robot({"c": "hb"})
            if self.last_tel and time.time() - self.last_tel > 2:
                self.broadcast({"t": "link", "ok": False})
            await asyncio.sleep(0.5)


class UdpProto(asyncio.DatagramProtocol):
    def __init__(self, app):
        self.app = app

    def connection_made(self, transport):
        self.app.transport = transport

    def datagram_received(self, data, addr):
        self.app.on_udp(data, addr)


async def ws_handler(request):
    app = request.app["mapper"]
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    app.browsers.add(ws)
    await ws.send_str(json.dumps(app.map_msg()))
    await ws.send_str(json.dumps({"t": "path", "pts": app.path}))
    await ws.send_str(json.dumps(app.cfg_msg()))
    await ws.send_str(json.dumps({"t": "scanpts", "pts": app.last_scan_pts}))
    if app.tel:
        await ws.send_str(json.dumps({"t": "tel", **app.tel, "link": time.time() - app.last_tel < 2}))
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    await app.on_browser(json.loads(msg.data), ws)
                except Exception as e:
                    app.log(f"ошибка команды: {e}")
    finally:
        app.browsers.discard(ws)
    return ws


async def index(request):
    return web.FileResponse(WEB_DIR / "index.html")


def local_ips():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ips.add(s.getsockname()[0])
        s.close()
    except Exception:
        pass
    return sorted(ips) or ["127.0.0.1"]


async def main():
    app = App()
    loop = asyncio.get_running_loop()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", C.PC_PORT))
    await loop.create_datagram_endpoint(lambda: UdpProto(app), sock=sock)

    wapp = web.Application()
    wapp["mapper"] = app
    wapp.router.add_get("/", index)
    wapp.router.add_get("/ws", ws_handler)
    runner = web.AppRunner(wapp)
    await runner.setup()
    port = int(os.environ.get("MAPPER_WEB_PORT", C.WEB_PORT))
    await web.TCPSite(runner, "0.0.0.0", port).start()
    print(f"mapper: UDP {C.PC_PORT}, веб-интерфейс:")
    for ip in local_ips():
        print(f"   http://{ip}:{port}")
    print("ждём машинку...", flush=True)
    await app.heartbeat()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
