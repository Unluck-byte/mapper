"""Поиск пути A* по карте с «раздутыми» стенами + упрощение маршрута."""
import heapq
import math

import numpy as np

from . import config as C
from .grid import Grid


def _disk(r):
    return [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1) if dx * dx + dy * dy <= r * r]


def inflate(mask, r):
    out = mask.copy()
    H, W = mask.shape
    ys, xs = np.nonzero(mask)
    for dy, dx in _disk(r):
        yy, xx = ys + dy, xs + dx
        ok = (yy >= 0) & (yy < H) & (xx >= 0) & (xx < W)
        out[yy[ok], xx[ok]] = True
    return out


def blocked_mask(grid):
    occ = grid.occ()
    r = math.ceil((C.ROBOT_RADIUS_MM + C.SAFETY_MARGIN_MM) / C.RES_MM)
    return inflate(occ, r)


def _line_free(blk, a, b):
    (c0, r0), (c1, r1) = a, b
    n = max(abs(c1 - c0), abs(r1 - r0))
    for k in range(n + 1):
        c = round(c0 + (c1 - c0) * k / max(n, 1))
        r = round(r0 + (r1 - r0) * k / max(n, 1))
        if blk[r, c]:
            return False
    return True


def plan(grid, start_mm, goal_mm, unknown_cost=2.5):
    """Возвращает (список точек [x,y] в мм без старта, сообщение) или (None, причина)."""
    blk = blocked_mask(grid)
    unknown = np.abs(grid.L) < 0.2
    s = Grid.to_cell(*start_mm)
    g = Grid.to_cell(*goal_mm)
    if not (Grid.inside(*s) and Grid.inside(*g)):
        return None, "точка вне карты"

    # машинка у стены — выпускаем её из «раздутой» зоны
    r0 = math.ceil((C.ROBOT_RADIUS_MM + C.SAFETY_MARGIN_MM) / C.RES_MM)
    for dy, dx in _disk(r0):
        y, x = s[1] + dy, s[0] + dx
        if 0 <= y < C.SIZE and 0 <= x < C.SIZE and not (grid.L[y, x] > C.OCC_T):
            blk[y, x] = False

    if blk[g[1], g[0]]:
        best = None
        for rad in range(1, 12):
            for dy, dx in _disk(rad):
                y, x = g[1] + dy, g[0] + dx
                if 0 <= y < C.SIZE and 0 <= x < C.SIZE and not blk[y, x]:
                    d = dx * dx + dy * dy
                    if best is None or d < best[0]:
                        best = (d, (x, y))
            if best:
                break
        if not best:
            return None, "точка внутри препятствия"
        g = best[1]

    # окно поиска: известная область + запас (быстрее)
    known = np.nonzero(~unknown)
    pad = 30
    cys = list(known[0]) + [s[1], g[1]]
    cxs = list(known[1]) + [s[0], g[0]]
    y0, y1 = max(0, min(cys) - pad), min(C.SIZE - 1, max(cys) + pad)
    x0, x1 = max(0, min(cxs) - pad), min(C.SIZE - 1, max(cxs) + pad)

    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]

    def h(c):
        dx, dy = abs(c[0] - g[0]), abs(c[1] - g[1])
        return (dx + dy) + (1.414 - 2) * min(dx, dy)

    openq = [(h(s), 0.0, s)]
    came = {s: None}
    cost = {s: 0.0}
    found = False
    while openq:
        _, gc, cur = heapq.heappop(openq)
        if cur == g:
            found = True
            break
        if gc > cost.get(cur, 1e18):
            continue
        cx, cy = cur
        for dx, dy, w in moves:
            nx, ny = cx + dx, cy + dy
            if not (x0 <= nx <= x1 and y0 <= ny <= y1) or blk[ny, nx]:
                continue
            if dx and dy and (blk[cy, nx] or blk[ny, cx]):
                continue
            nc = gc + w * (unknown_cost if unknown[ny, nx] else 1.0)
            if nc < cost.get((nx, ny), 1e18):
                cost[(nx, ny)] = nc
                came[(nx, ny)] = cur
                heapq.heappush(openq, (nc + h((nx, ny)), nc, (nx, ny)))
    if not found:
        return None, "пути нет"

    cells = []
    c = g
    while c is not None:
        cells.append(c)
        c = came[c]
    cells.reverse()

    # упрощение: оставляем только точки поворота с прямой видимостью
    simp = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not _line_free(blk, cells[i], cells[j]):
            j -= 1
        simp.append(cells[j])
        i = j
    pts = [list(map(lambda v: round(v, 1), Grid.to_world(*c))) for c in simp[1:]]
    if pts:
        pts[-1] = [round(goal_mm[0], 1), round(goal_mm[1], 1)] if g == Grid.to_cell(*goal_mm) else pts[-1]
    return pts, f"{len(pts)} точек"


def ahead_blocked(grid, x, y, th, dist_mm):
    """Есть ли стена карты на пути вперёд на dist_mm (для ручной езды)."""
    occ = grid.occ()
    r = C.ROBOT_RADIUS_MM
    for d in range(0, int(dist_mm) + 1, C.RES_MM):
        for side in (-r, -r * 0.5, 0, r * 0.5, r):
            px = x + (r + d) * math.cos(th) - side * math.sin(th)
            py = y + (r + d) * math.sin(th) + side * math.cos(th)
            c = Grid.to_cell(px, py)
            if Grid.inside(*c) and occ[c[1], c[0]]:
                return True
    return False
