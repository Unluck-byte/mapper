"""Симулятор машинки: тот же UDP-протокол, что у прошивки. Позволяет проверить
карту, ICP, маршруты и интерфейс без железа.

Запуск (из папки pc/):  .venv/bin/python -m mapper.sim
Потом сервер:           .venv/bin/python -m mapper.server
"""
import asyncio
import json
import math
import random
import socket
import time

from . import config as C

# комната (мм): стены как отрезки. Машинка стартует в (0,0), смотрит по +x.
ROOM = [(-800, -1500), (4200, -1500), (4200, 2000), (-800, 2000)]
OBSTACLES = [
    [(1400, -400), (1900, -400), (1900, 300), (1400, 300)],          # коробка
    [(2600, 900), (3300, 900), (3300, 1300), (2600, 1300)],          # диван
    [(-800, 900), (300, 900), (300, 1000), (-800, 1000)],            # перегородка
]

ROBOT_R = C.ROBOT_RADIUS_MM - 20
TRACK = 150.0
DT = 0.02


def segments():
    segs = []
    for poly in [ROOM] + OBSTACLES:
        for i in range(len(poly)):
            segs.append((poly[i], poly[(i + 1) % len(poly)]))
    return segs


SEGS = segments()


def ray_cast(x, y, a, max_d=6000):
    dx, dy = math.cos(a), math.sin(a)
    best = max_d
    for (x1, y1), (x2, y2) in SEGS:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((x1 - x) * ey - (y1 - y) * ex) / den
        u = ((x1 - x) * dy - (y1 - y) * dx) / den
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


def collides(x, y):
    for (x1, y1), (x2, y2) in SEGS:
        ex, ey = x2 - x1, y2 - y1
        L2 = ex * ex + ey * ey
        u = max(0, min(1, ((x - x1) * ex + (y - y1) * ey) / L2))
        if math.hypot(x - (x1 + u * ex), y - (y1 + u * ey)) < ROBOT_R:
            return True
    return False


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Sim:
    def __init__(self):
        self.gx = self.gy = self.gth = 0.0          # истинное положение
        self.ox = self.oy = self.oth = 0.0          # одометрия (то, что «думает» машинка)
        self.v = self.w = 0.0
        self.mode = "idle"
        self.block = ""
        self.man = (0.0, 0.0, 0.0)
        self.wp = []
        self.wpi = 0
        self.rot = True
        self.turn = 0.0
        self.task = 0
        self.scan = None
        self.pc = ("127.0.0.1", C.PC_PORT)
        self.events = []
        self.drift = random.uniform(-0.002, 0.002)    # рад/с — «уплывание» гироскопа в движении (0.1°/с)
        self.slip = random.uniform(0.97, 1.03)      # ошибка масштаба энкодеров

    # --- датчики
    def tof(self, a_rel):
        sx = self.gx + C.TOF_POS_X_MM * math.cos(self.gth)
        sy = self.gy + C.TOF_POS_X_MM * math.sin(self.gth)
        a = self.gth + a_rel
        half = math.radians(C.BEAM_HALF_DEG)                                          # широкий луч:
        d = min(ray_cast(sx, sy, a + k * half / 4) for k in range(-4, 5))            # ближайшее в конусе
        if d > C.TOF_MAX_MM:
            return 0
        return max(40, int(d + random.gauss(0, 12)))

    def us(self, back=False):
        a = self.gth + (math.pi if back else 0)
        d = min(ray_cast(self.gx, self.gy, a + k * math.radians(7.5)) for k in (-2, -1, 0, 1, 2)) - ROBOT_R   # конус ±15°
        return int(max(20, min(3000, d)))

    # --- команды
    def cmd(self, m):
        c = m.get("c")
        if c == "drive" and self.mode != "scan":
            self.man = (m.get("v", 0), m.get("w", 0), time.time())
            self.mode = "manual"
        elif c == "stop":
            self.stop("cmd")
        elif c == "scan":
            self.task = m.get("id", 0)
            self.mode = "scan"
            self.scan = {"ph": "settle", "t": time.time(), "half": 0, "ang": 0, "pts": [], "x": self.ox, "y": self.oy}
        elif c == "path":
            self.wp = m["pts"]
            self.wpi = 0
            self.rot = True
            self.task = m.get("id", 0)
            self.mode = "path"
        elif c == "turn":
            self.turn = m["th"]
            self.task = m.get("id", 0)
            self.mode = "turn"
        elif c == "pose":
            self.ox, self.oy, self.oth = m["x"], m["y"], m["th"]

    def stop(self, why):
        if self.mode in ("path", "turn", "scan"):
            self.events.append({"t": "ev", "ev": "stopped", "id": self.task, "why": why})
        self.mode = "idle"
        self.v = self.w = 0

    # --- шаг 50 Гц
    def step(self):
        vt = wt = 0.0
        now = time.time()
        if self.mode == "manual":
            v, w, t = self.man
            if now - t < 0.5:
                vt, wt = v, w
            else:
                self.mode = "idle"
        elif self.mode == "turn":
            err = wrap(self.turn - self.oth)
            wt = max(-1.6, min(1.6, 2.5 * err))
            if abs(err) < 0.04:
                self.mode = "idle"
                self.events.append({"t": "ev", "ev": "turn_done", "id": self.task})
        elif self.mode == "path":
            tx, ty = self.wp[self.wpi]
            dx, dy = tx - self.ox, ty - self.oy
            dist = math.hypot(dx, dy)
            last = self.wpi == len(self.wp) - 1
            if dist < (40 if last else 70):
                self.wpi += 1
                self.rot = True
                if self.wpi >= len(self.wp):
                    self.mode = "idle"
                    self.events.append({"t": "ev", "ev": "path_done", "id": self.task})
            else:
                err = wrap(math.atan2(dy, dx) - self.oth)
                if self.rot:
                    wt = max(-1.6, min(1.6, 2.5 * err))
                    if abs(err) < 0.07:
                        self.rot = False
                elif abs(err) > 0.5:
                    self.rot = True
                else:
                    vt = min(180, 1.5 * dist + 50)
                    wt = max(-0.8, min(0.8, 2.0 * err))
        elif self.mode == "scan":
            vt, wt = self.scan_step(now)

        # защита (как в прошивке)
        blk = ""
        lim = 100 if self.mode == "path" else 180
        tof_fwd = self.tof(0) if self.mode != "scan" else 0
        if vt > 0 and (self.us() < lim or (tof_fwd and tof_fwd < lim)):
            vt, blk = 0, "front"
        if vt < 0 and self.us(True) < 150:
            vt, blk = 0, "rear"
        if blk and self.mode == "path":
            self.stop(blk)
        if blk and blk != self.block:
            self.events.append({"t": "ev", "ev": "blocked", "why": blk})
        self.block = blk

        self.v += max(-20, min(10, vt - self.v)) if not blk else -self.v
        self.w += max(-0.12, min(0.12, wt - self.w))

        # истинное движение (+ столкновения)
        nth = wrap(self.gth + self.w * DT)
        nx = self.gx + self.v * DT * math.cos(self.gth)
        ny = self.gy + self.v * DT * math.sin(self.gth)
        moved = True
        if not collides(nx, ny):
            self.gx, self.gy = nx, ny
        else:
            moved = False
        self.gth = nth
        # одометрия с ошибками
        ds = (self.v * DT * self.slip) if moved else self.v * DT * 0.6
        drift = self.drift * DT if (abs(self.v) > 1 or abs(self.w) > 0.01) else 0
        self.oth = wrap(self.oth + self.w * DT + drift)
        self.ox += ds * math.cos(self.oth)
        self.oy += ds * math.sin(self.oth)

    def scan_step(self, now):
        s = self.scan
        if s["ph"] == "settle":
            if now - s["t"] > 0.3:
                s["ph"] = "sweep"
                s["ang"] = 0 if s["half"] == 0 else 180
                s["t"] = now
        elif s["ph"] == "sweep":
            if now - s["t"] >= 0.012:
                s["t"] = now
                rel = math.radians(s["ang"] - 90)
                s["pts"].append([int(self.oth * 1000), int((s["ang"] - 90) * 10), self.tof(rel)])
                nxt = s["ang"] + (3 if s["half"] == 0 else -3)
                if 0 <= nxt <= 180:
                    s["ang"] = nxt
                elif s["half"] == 0:
                    s["half"] = 1
                    s["target"] = wrap(self.oth + math.pi)
                    s["ph"] = "rotate"
                else:
                    self.mode = "idle"
                    self.events.append({"t": "ev", "ev": "scan_done", "id": self.task, "n": len(s["pts"])})
                    self.events.append(("SCAN", s))
        elif s["ph"] == "rotate":
            err = wrap(s["target"] - self.oth)
            if abs(err) < 0.05:
                s["ph"] = "settle"
                s["t"] = now
                return 0, 0
            return 0, max(-1.6, min(1.6, 2.5 * err))
        return 0, 0

    def telemetry(self):
        sv = 90
        if self.mode == "scan" and self.scan["ph"] == "sweep":
            sv = self.scan["ang"]
        return {"t": "tel", "x": round(self.ox, 1), "y": round(self.oy, 1), "th": round(self.oth, 4),
                "vl": round(self.v - self.w * TRACK / 2), "vr": round(self.v + self.w * TRACK / 2),
                "v": round(self.v), "w": round(self.w, 2), "mode": self.mode, "blk": self.block,
                "wp": self.wpi, "wpn": len(self.wp), "id": self.task, "usf": self.us(), "usr": self.us(True),
                "cliff": 0, "tof": self.tof(0), "servo": sv, "bat": 11700, "imu": 1, "tofok": 1, "rssi": -50,
                "up": 0, "sim": True, "gt": [round(self.gx, 1), round(self.gy, 1), round(self.gth, 4)]}


class Proto(asyncio.DatagramProtocol):
    def __init__(self, sim):
        self.sim = sim

    def datagram_received(self, data, addr):
        try:
            m = json.loads(data.decode())
        except Exception:
            return
        self.sim.pc = (addr[0], C.PC_PORT)
        self.sim.cmd(m)


async def main():
    sim = Sim()
    loop = asyncio.get_running_loop()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", C.ROBOT_PORT))
    tr, _ = await loop.create_datagram_endpoint(lambda: Proto(sim), sock=sock)
    print(f"симулятор: слушаю {C.ROBOT_PORT}, шлю на 127.0.0.1:{C.PC_PORT}", flush=True)

    def send(m):
        tr.sendto(json.dumps(m).encode(), sim.pc)

    k = 0
    while True:
        sim.step()
        for e in sim.events:
            if isinstance(e, tuple):
                s = e[1]
                pts = s["pts"]
                parts = (len(pts) + 39) // 40
                for i in range(parts):
                    send({"t": "scan", "id": sim.task, "part": i, "parts": parts, "x": round(s["x"], 1),
                          "y": round(s["y"], 1), "pts": pts[i * 40:(i + 1) * 40]})
            else:
                send(e)
        sim.events.clear()
        if k % 5 == 0:
            send(sim.telemetry())
        k += 1
        await asyncio.sleep(DT)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
