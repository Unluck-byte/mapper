"""Карта-сетка занятости (лог-шансы)."""
import base64
import numpy as np

from . import config as C


class Grid:
    def __init__(self):
        self.L = np.zeros((C.SIZE, C.SIZE), dtype=np.float32)   # [row=y, col=x]
        self.version = 0
        self.noise_level = C.NOISE_LEVEL
        self._occ_cache = (None, None)

    # --- координаты: мм <-> клетка
    @staticmethod
    def to_cell(x, y):
        return int(np.floor(x / C.RES_MM)) + C.SIZE // 2, int(np.floor(y / C.RES_MM)) + C.SIZE // 2

    @staticmethod
    def to_world(cx, cy):
        return (cx - C.SIZE // 2 + 0.5) * C.RES_MM, (cy - C.SIZE // 2 + 0.5) * C.RES_MM

    @staticmethod
    def inside(cx, cy):
        return 0 <= cx < C.SIZE and 0 <= cy < C.SIZE

    def clear(self):
        self.L[:] = 0
        self.version += 1

    # --- обновление по лучу (Брезенхем)
    def _ray(self, x0, y0, x1, y1, hit):
        c0, r0 = self.to_cell(x0, y0)
        c1, r1 = self.to_cell(x1, y1)
        dc, dr = abs(c1 - c0), -abs(r1 - r0)
        sc, sr = (1 if c0 < c1 else -1), (1 if r0 < r1 else -1)
        err = dc + dr
        c, r = c0, r0
        L = self.L
        while True:
            if c == c1 and r == r1:
                break
            if self.inside(c, r):
                L[r, c] = max(C.L_MIN, L[r, c] + C.L_MISS)
            e2 = 2 * err
            if e2 >= dr:
                err += dr
                c += sc
            if e2 <= dc:
                err += dc
                r += sr
        if hit and self.inside(c1, r1):
            L[r1, c1] = min(C.L_MAX, L[r1, c1] + C.L_HIT)

    def integrate(self, rays):
        """rays: список (sx, sy, ex, ey, hit) в мм."""
        for sx, sy, ex, ey, hit in rays:
            self._ray(sx, sy, ex, ey, hit)
        self.version += 1

    # --- фильтр помех
    def occ(self):
        """Маска стен без одиночных помех: у клетки стены должно быть ≥ noise_level соседей-стен
        в радиусе NOISE_RADIUS_MM (квадрат). Сама карта не меняется — фильтр только при чтении."""
        key = (self.version, self.noise_level)
        if self._occ_cache[0] == key:
            return self._occ_cache[1]
        occ = self.L > C.OCC_T
        if self.noise_level > 0 and occ.any():
            r = max(1, round(C.NOISE_RADIUS_MM / C.RES_MM))
            ii = np.pad(occ.astype(np.int32), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
            n = C.SIZE
            y0 = np.clip(np.arange(n) - r, 0, n); y1 = np.clip(np.arange(n) + r + 1, 0, n)
            box = ii[y1][:, y1] - ii[y0][:, y1] - ii[y1][:, y0] + ii[y0][:, y0]
            occ = occ & (box - 1 >= self.noise_level)
        self._occ_cache = (key, occ)
        return occ

    def clear_trail(self, x, y, radius_mm=C.TRAIL_FREE_MM):
        """Машинка здесь стоит/проехала — клетки под ней свободны."""
        c, r = self.to_cell(x, y)
        k = int(radius_mm / C.RES_MM)
        changed = False
        for dy in range(-k, k + 1):
            for dx in range(-k, k + 1):
                if dx * dx + dy * dy <= k * k and self.inside(c + dx, r + dy):
                    v = self.L[r + dy, c + dx]
                    if v > C.FREE_T:
                        self.L[r + dy, c + dx] = C.FREE_T - 0.4
                        changed = True
        if changed:
            self.version += 1

    # --- выборки
    def occupied_points(self, cx_mm=None, cy_mm=None, radius_mm=None):
        rows, cols = np.nonzero(self.L > C.OCC_T)
        xs = (cols - C.SIZE // 2 + 0.5) * C.RES_MM
        ys = (rows - C.SIZE // 2 + 0.5) * C.RES_MM
        pts = np.stack([xs, ys], axis=1)
        if radius_mm is not None and len(pts):
            d = np.hypot(pts[:, 0] - cx_mm, pts[:, 1] - cy_mm)
            pts = pts[d < radius_mm]
        return pts

    def encoded(self):
        """uint8: 0 — неизвестно … 128 — середина … 255 — стена. base64 для браузера."""
        L = self.L
        if self.noise_level > 0:
            noise = (L > C.OCC_T) & ~self.occ()
            if noise.any():
                L = L.copy()
                L[noise] = 0.0            # помеха показывается как «неизвестно»
        v = np.clip(128 + L * 31.75, 0, 255).astype(np.uint8)
        return base64.b64encode(v.tobytes()).decode()

    def save(self, path):
        np.save(path, self.L)

    def load(self, path):
        L = np.load(path)
        if L.shape == self.L.shape:
            self.L[:] = L
            self.version += 1
            return True
        return False
