"""Совмещение нового скана с картой: корреляционный перебор (CSM) + точная доводка.

Перебираем сдвиг/поворот машинки вокруг положения по одометрии и выбираем тот,
при котором точки скана ложатся на стены карты лучше всего («поле правдоподобия»).
Устойчив к частичному перекрытию (новое место видит то, чего на карте ещё нет)."""
import math

import numpy as np

from . import config as C


def likelihood_field(L, sigma_mm=60.0, max_cells=12):
    """Для каждой клетки: exp(-d²/2σ²), d — расстояние до ближайшей стены."""
    occ = L > C.OCC_T
    dist = np.full(L.shape, max_cells, dtype=np.float32)
    dist[occ] = 0
    cur = occ.copy()
    for k in range(1, max_cells):
        grown = cur.copy()
        grown[1:, :] |= cur[:-1, :]
        grown[:-1, :] |= cur[1:, :]
        grown[:, 1:] |= cur[:, :-1]
        grown[:, :-1] |= cur[:, 1:]
        grown[1:, 1:] |= cur[:-1, :-1]
        grown[:-1, :-1] |= cur[1:, 1:]
        grown[1:, :-1] |= cur[:-1, 1:]
        grown[:-1, 1:] |= cur[1:, :-1]
        new = grown & ~cur
        dist[new] = k
        cur = grown
    d_mm = dist * C.RES_MM
    return np.exp(-(d_mm ** 2) / (2 * sigma_mm ** 2)).astype(np.float32)


def _score(field, pts_rel, ox, oy, th_list, txy):
    """pts_rel (N,2) — точки относительно машинки; возвращает (len(th), len(txy)) суммы."""
    half = C.SIZE // 2
    out = np.empty((len(th_list), len(txy)), dtype=np.float32)
    for i, th in enumerate(th_list):
        c, s = math.cos(th), math.sin(th)
        q = pts_rel @ np.array([[c, s], [-s, c]], dtype=np.float32)       # поворот на th
        X = ox + q[None, :, 0] + txy[:, None, 0]
        Y = oy + q[None, :, 1] + txy[:, None, 1]
        cx = np.floor(X / C.RES_MM).astype(np.int32) + half
        cy = np.floor(Y / C.RES_MM).astype(np.int32) + half
        ok = (cx >= 0) & (cx < C.SIZE) & (cy >= 0) & (cy < C.SIZE)
        v = np.zeros(cx.shape, dtype=np.float32)
        v[ok] = field[cy[ok], cx[ok]]
        out[i] = v.sum(axis=1)
    return out


def match(L, hits, ox, oy, max_xy=C.ICP_MAX_SHIFT_MM, max_th=C.ICP_MAX_ROT_RAD):
    max_xy = max(50, min(max_xy, C.ICP_MAX_SHIFT_MM))
    """hits (N,2) — точки скана в мире по одометрии, (ox,oy) — положение машинки.
    Возвращает dict(dx, dy, dth, score) — поправку вокруг (ox,oy), или None."""
    if len(hits) < 10:
        return None
    field = likelihood_field(L)
    rel = (np.asarray(hits, dtype=np.float32) - np.array([ox, oy], dtype=np.float32))

    def grid_search(cx, cy, cth, xy_half, xy_step, th_half, th_step):
        ths = cth + np.arange(-th_half, th_half + 1e-9, th_step)
        r = np.arange(-xy_half, xy_half + 1e-9, xy_step)
        txy = np.array([(cx + a, cy + b) for a in r for b in r], dtype=np.float32)
        sc = _score(field, rel, ox, oy, ths, txy)
        i, j = np.unravel_index(np.argmax(sc), sc.shape)
        return float(txy[j, 0]), float(txy[j, 1]), float(ths[i]), float(sc[i, j])

    # грубо → точнее → ещё точнее
    dx, dy, dth, _ = grid_search(0, 0, 0, max_xy, 50, max_th, math.radians(2))
    dx, dy, dth, _ = grid_search(dx, dy, dth, 50, 12.5, math.radians(2), math.radians(0.5))
    dx, dy, dth, sc = grid_search(dx, dy, dth, 12.5, 4, math.radians(0.5), math.radians(0.125))
    base = _score(field, rel, ox, oy, [0.0], np.zeros((1, 2), dtype=np.float32))[0, 0]
    n = len(rel)

    # насколько скан «держит» положение в каждом направлении (кривизна пика качества).
    # Вдоль длинной стены/коридора пик плоский — туда положение НЕ исправляем.
    d = 60.0
    offs = np.array([[0, 0], [d, 0], [-d, 0], [0, d], [0, -d], [d, d], [-d, -d], [d, -d], [-d, d]], dtype=np.float32)
    v = _score(field, rel, ox, oy, [dth], offs + np.array([dx, dy], dtype=np.float32))[0] / n
    hxx = (v[1] + v[2] - 2 * v[0]) / d ** 2
    hyy = (v[3] + v[4] - 2 * v[0]) / d ** 2
    hxy = (v[5] + v[6] - v[7] - v[8]) / (4 * d ** 2)
    curv, vecs = np.linalg.eigh(-np.array([[hxx, hxy], [hxy, hyy]]))
    corr = np.array([dx, dy])
    kept = np.zeros(2)
    weak = []
    for k in range(2):
        drop = curv[k] * d ** 2                  # падение качества при сдвиге на 6 см
        comp = vecs[:, k] @ corr
        if drop >= C.MATCH_MIN_DROP:
            kept += comp * vecs[:, k]
        else:
            weak.append(round(float(drop), 3))
    return {"dx": float(kept[0]), "dy": float(kept[1]), "dth": dth, "score": sc / n, "base": float(base) / n,
            "raw_dx": dx, "raw_dy": dy, "weak": weak}


def beam_correct(raw, half_deg=C.BEAM_HALF_DEG, step_deg=C.SCAN_STEP_DEG):
    """Поправка широкого луча. TOF возвращает расстояние до БЛИЖАЙШЕЙ точки в конусе;
    у наклонной стены она лежит у края конуса, а не по центру. По соседним замерам
    видно, растёт расстояние или падает, — сдвигаем точку к краю конуса.
    raw: [(курс_мрад, угол_сервы_0.1°, мм)] -> [(курс_рад, угол_рад, мм)]"""
    c = math.radians(half_deg)
    step = math.radians(step_deg)
    pts = [(t / 1000.0, math.radians(r / 10.0), mm) for t, r, mm in raw]
    out = []
    for i, (th, rel, mm) in enumerate(pts):
        shift = 0.0
        if mm and c > 0:
            nb = [j for j in (i - 1, i + 1) if 0 <= j < len(pts) and pts[j][2]
                  and abs(pts[j][0] - th) < 0.05 and abs(abs(pts[j][1] - rel) - step) < 0.01]
            if len(nb) == 2:
                (a1, d1), (a2, d2) = sorted((pts[j][1], pts[j][2]) for j in nb)
                if abs(d2 - d1) < 0.5 * mm:                  # одна поверхность, не край предмета
                    s = (d2 - d1) / (a2 - a1) / mm           # = tan(угла падения)
                    shift = -c * max(-1.0, min(1.0, s / math.tan(c)))
        out.append((th, rel + shift, mm))
    return out


def edge_filter(hits_with_d, jump_mm=150, max_mm=3000):
    """Убирает точки на краях предметов (там широкий луч TOF «размазывает») и дальние."""
    d = [h[2] for h in hits_with_d]
    out = []
    for i, h in enumerate(hits_with_d):
        a = abs(d[i] - d[i - 1]) if i > 0 else 0
        b = abs(d[i] - d[i + 1]) if i < len(d) - 1 else 0
        if max(a, b) < jump_mm and d[i] < max_mm:
            out.append(h[:2])
    return out
