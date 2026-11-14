#!/usr/bin/env python3
"""Генератор схемы подключения SLAM-машинки mapper (SVG)."""
import sys
from itertools import combinations

W, H = 3300, 2120
out = []
FONT = "Noto Sans, Liberation Sans, sans-serif"
MONO = "JetBrainsMono NF, Noto Sans Mono, monospace"

def add(s): out.append(s)

def esc(t):
    return str(t).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

def text(x, y, t, size=14, fill="#e9ecef", anchor="start", weight="normal", font=FONT, rot=None, italic=False):
    tr = f' transform="rotate({rot} {x} {y})"' if rot is not None else ""
    st = ' font-style="italic"' if italic else ""
    add(f'<text x="{x}" y="{y}" font-family="{font}" font-size="{size}" fill="{fill}" '
        f'text-anchor="{anchor}" font-weight="{weight}"{st}{tr}>{esc(t)}</text>')

def rect(x, y, w, h, fill, stroke="none", sw=1, rx=0, extra=""):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" {extra}/>')

def circle(x, y, r, fill, stroke="none", sw=1):
    add(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

def line(x1, y1, x2, y2, stroke, sw=1, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    add(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{stroke}" stroke-width="{sw}"{d}/>')

def wire(pts, color, w=4, dot_end=True, dot_start=True):
    d = "M " + " L ".join(f"{x} {y}" for x, y in pts)
    outline = "#0a0b0f" if color not in ("#212529",) else "#9aa0a6"
    add(f'<path d="{d}" fill="none" stroke="{outline}" stroke-width="{w+3}" stroke-linejoin="round" stroke-linecap="round"/>')
    add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{w}" stroke-linejoin="round" stroke-linecap="round"/>')
    if dot_start:
        circle(*pts[0], w/2+2, color, outline, 1)
    if dot_end:
        circle(*pts[-1], w/2+2, color, outline, 1)

def junction(x, y, color):
    circle(x, y, 5, color, "#0a0b0f", 1.5)

NET = {"12V": "#e03131", "7V": "#fd7e14", "5V": "#ff6b6b", "3V3": "#fcc419", "GND": "#adb5bd"}

def tag(x, y, net, label=None, anchor="middle"):
    """Цветная метка питания (net label)."""
    lab = label or net
    tw = 9 * len(lab) + 14
    if anchor == "middle":
        x0 = x - tw / 2
    elif anchor == "start":
        x0 = x
    else:
        x0 = x - tw
    c = NET.get(net, "#ced4da")
    rect(x0, y - 11, tw, 22, "#101217", c, 2, 11)
    text(x0 + tw / 2, y + 5, lab, 13, c, "middle", "bold")

def gnd(x, y, down=True):
    """Символ земли: линия от (x,y) вниз и три черты."""
    c = NET["GND"]
    s = 1 if down else -1
    line(x, y, x, y + 8 * s, c, 2.5)
    line(x - 9, y + 8 * s, x + 9, y + 8 * s, c, 2.5)
    line(x - 6, y + 12 * s, x + 6, y + 12 * s, c, 2.5)
    line(x - 3, y + 16 * s, x + 3, y + 16 * s, c, 2.5)

def stub_tag(px, py, net, dx=0, dy=22, label=None):
    """Короткий провод от пина к метке питания."""
    c = NET[net]
    wire([(px, py), (px + dx, py + dy)], c, 3, dot_end=False)
    tag(px + dx, py + dy + (11 if dy > 0 else -11) if dx == 0 else py + dy, net, label,
        "middle" if dx == 0 else ("start" if dx > 0 else "end"))

def pin(x, y, size=9):
    rect(x - size / 2, y - size / 2, size, size, "#d4af37", "#7a5c00", 1, 1.5)

def resistor(x, y, vertical=False, label="", bands=("#8b4513", "#000", "#e03131")):
    """Резистор с цветными полосками; (x,y) — центр."""
    if vertical:
        rect(x - 7, y - 20, 14, 40, "#e3c99a", "#7d6640", 1, 6)
        for i, b in enumerate(bands):
            rect(x - 7, y - 13 + i * 8, 14, 4, b)
        if label:
            text(x + 14, y + 5, label, 13, "#f1f3f5", "start", "bold")
    else:
        rect(x - 20, y - 7, 40, 14, "#e3c99a", "#7d6640", 1, 6)
        for i, b in enumerate(bands):
            rect(x - 13 + i * 8, y - 7, 4, 14, b)
        if label:
            text(x, y - 13, label, 13, "#f1f3f5", "middle", "bold")

# ---------------------------------------------------------------- фон
add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
add('<defs>'
    '<pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse">'
    '<path d="M 40 0 L 0 0 0 40" fill="none" stroke="#1d2129" stroke-width="1"/></pattern>'
    '<linearGradient id="shield" x1="0" y1="0" x2="1" y2="1">'
    '<stop offset="0" stop-color="#e9ecef"/><stop offset="0.5" stop-color="#adb5bd"/><stop offset="1" stop-color="#dee2e6"/></linearGradient>'
    '<linearGradient id="can" x1="0" y1="0" x2="0" y2="1">'
    '<stop offset="0" stop-color="#f1f3f5"/><stop offset="0.5" stop-color="#868e96"/><stop offset="1" stop-color="#ced4da"/></linearGradient>'
    '<radialGradient id="lens" cx="0.4" cy="0.35" r="0.7">'
    '<stop offset="0" stop-color="#5c7cfa"/><stop offset="0.5" stop-color="#1c1e3a"/><stop offset="1" stop-color="#05060a"/></radialGradient>'
    '<radialGradient id="ustx" cx="0.5" cy="0.5" r="0.5">'
    '<stop offset="0" stop-color="#495057"/><stop offset="0.6" stop-color="#ced4da"/><stop offset="1" stop-color="#f8f9fa"/></radialGradient>'
    '</defs>')
rect(0, 0, W, H, "#14161c")
rect(0, 0, W, H, "url(#grid)")

text(40, 62, "MAPPER — схема подключения SLAM-машинки", 40, "#f8f9fa", weight="bold")
text(40, 100, "ESP32-S3-DevKitC-1 N16R8 · TOF400F на серве · 4× TT-мотор + энкодеры LM393 · MX1508 · MPU6050 · HC-SR04 ×2 · ИК пола · 3S 18650",
     19, "#adb5bd")
text(40, 132, "Все GND — общие.  Сигналы нарисованы проводами, питание — цветными метками (3V3 / 5V / 7V / GND).  Номера на ESP = номера GPIO (подписи на плате).",
     17, "#868e96")

# ---------------------------------------------------------------- ESP32-S3
BX, BY, BW, BH = 1260, 200, 340, 1090
P0, PITCH = 390, 38
LX, RX_ = BX + 22, BX + BW - 22
LEFT = ["3V3", "3V3", "RST", "4", "5", "6", "7", "15", "16", "17", "18", "8", "3", "46", "9", "10", "11", "12", "13", "14", "5V", "GND"]
RIGHT = ["GND", "TX", "RX", "1", "2", "42", "41", "40", "39", "38", "37", "36", "35", "0", "45", "48", "47", "21", "20", "19", "GND", "GND"]

def ly(name, nth=0):
    idx = [i for i, n in enumerate(LEFT) if n == name][nth]
    return P0 + idx * PITCH

def ry(name, nth=0):
    idx = [i for i, n in enumerate(RIGHT) if n == name][nth]
    return P0 + idx * PITCH

FUNC_L = {"3V3": ("3V3 → датчики", NET["3V3"]), "4": ("SERVO", "#ff922b"), "5": ("TOF → RXD", "#f1f3f5"),
          "6": ("TOF ← TXD", "#51cf66"), "7": ("US-F ECHO", "#ff8787"), "15": ("US-F TRIG", "#ffd43b"),
          "16": ("US-R ECHO", "#f06595"), "17": ("US-R TRIG", "#c0eb75"), "18": ("ИК ПОЛ", "#63e6be"),
          "8": ("MX IN3", "#f783ac"), "9": ("MX IN4", "#748ffc"), "10": ("MX IN1", "#da77f2"),
          "11": ("MX IN2", "#9775fa"), "12": ("свободен", "#5c636a"), "13": ("ENC FL", "#3bc9db"),
          "14": ("ENC RL", "#a5d8ff"), "5V": ("5V ВХОД", NET["5V"]), "GND": ("GND", NET["GND"]),
          "3": ("не трогать", "#5c636a"), "46": ("не трогать", "#5c636a"), "RST": ("", "#5c636a")}
FUNC_R = {"1": ("BAT АЦП·опц.", "#d0bfff"), "2": ("ENC FR", "#38d9a9"), "42": ("MPU SCL", "#339af0"), "41": ("MPU SDA", "#ffe066"),
          "21": ("ENC RR", "#96f2d7"), "48": ("RGB LED", "#868e96"), "GND": ("GND", NET["GND"]),
          "0": ("BOOT·нет", "#5c636a"), "45": ("не трогать", "#5c636a"), "35": ("PSRAM·нет", "#5c636a"),
          "36": ("PSRAM·нет", "#5c636a"), "37": ("PSRAM·нет", "#5c636a"), "19": ("USB D−", "#5c636a"),
          "20": ("USB D+", "#5c636a"), "TX": ("UART0", "#5c636a"), "RX": ("UART0", "#5c636a")}

rect(BX, BY, BW, BH, "#0f1115", "#3d4350", 2, 16)
# модуль WROOM
rect(BX + 40, BY + 14, BW - 80, 162, "#1d2430", "#3d4350", 1, 4)
zz = " ".join(f"{BX + 60 + i * 22},{BY + 26 if i % 2 == 0 else BY + 52}" for i in range(11))
add(f'<polyline points="{zz}" fill="none" stroke="#c9a227" stroke-width="3"/>')
rect(BX + 52, BY + 64, BW - 104, 104, "url(#shield)", "#868e96", 1, 3)
text(BX + BW / 2, BY + 104, "ESP32-S3-WROOM-1", 17, "#212529", "middle", "bold")
text(BX + BW / 2, BY + 128, "N16R8 · 16 МБ flash · 8 МБ PSRAM", 13, "#343a40", "middle")
text(BX + BW / 2, BY + 152, "антенна — вверх, не закрывать металлом", 11, "#495057", "middle", italic=True)
# кнопки, LED, USB
rect(BX + 40, BY + BH - 78, 34, 24, "#adb5bd", "#495057", 1, 3); circle(BX + 57, BY + BH - 66, 7, "#212529")
text(BX + 57, BY + BH - 84, "BOOT", 11, "#adb5bd", "middle")
rect(BX + BW - 74, BY + BH - 78, 34, 24, "#adb5bd", "#495057", 1, 3); circle(BX + BW - 57, BY + BH - 66, 7, "#212529")
text(BX + BW - 57, BY + BH - 84, "RST", 11, "#adb5bd", "middle")
rect(BX + BW / 2 - 9, BY + BH - 76, 18, 18, "#f8f9fa", "#495057", 1, 2)
text(BX + BW / 2, BY + BH - 84, "RGB·48", 11, "#adb5bd", "middle")
for ux in (BX + 70, BX + BW - 140):
    rect(ux, BY + BH - 30, 70, 34, "#ced4da", "#495057", 1.5, 10)
    rect(ux + 12, BY + BH - 18, 46, 10, "#343a40", rx=5)
text(BX + 105, BY + BH + 24, "USB-C", 12, "#868e96", "middle")
text(BX + BW - 105, BY + BH + 24, "USB-C", 12, "#868e96", "middle")
text(BX + BW / 2, BY + BH + 50, "ESP32-S3-DevKitC-1", 22, "#f8f9fa", "middle", "bold")

for i, n in enumerate(LEFT):
    y = P0 + i * PITCH
    f, c = FUNC_L.get(n, ("", "#5c636a"))
    used = c != "#5c636a"
    pin(LX, y, 12)
    if used:
        circle(LX, y, 10, "none", c, 2)
    text(LX + 18, y + 5, n, 15, "#f8f9fa" if used else "#6c757d", "start", "bold", MONO)
    if f:
        text(LX + 58, y + 5, f, 12, c, "start", "bold" if used else "normal")
for i, n in enumerate(RIGHT):
    y = P0 + i * PITCH
    f, c = FUNC_R.get(n, ("", "#5c636a"))
    used = c != "#5c636a"
    pin(RX_, y, 12)
    if used:
        circle(RX_, y, 10, "none", c, 2)
    text(RX_ - 18, y + 5, n, 15, "#f8f9fa" if used else "#6c757d", "end", "bold", MONO)
    if f:
        text(RX_ - 58, y + 5, f, 12, c, "end", "bold" if used else "normal")

# питание ESP — метки
wire([(LX, ly("3V3")), (LX - 46, ly("3V3"))], NET["3V3"], 3, dot_end=False)
tag(LX - 46, ly("3V3"), "3V3", "3V3 выход", "end")
wire([(LX, ly("5V")), (LX - 46, ly("5V"))], NET["5V"], 3, dot_end=False)
tag(LX - 46, ly("5V"), "5V", "5V ← SS34", "end")
wire([(LX, ly("GND")), (LX - 30, ly("GND"))], NET["GND"], 3, dot_end=False)
gnd(LX - 30, ly("GND"))
wire([(RX_, ry("GND")), (RX_ + 30, ry("GND"))], NET["GND"], 3, dot_end=False)
gnd(RX_ + 30, ry("GND"))

# ---------------------------------------------------------------- модули слева
LEFT_WIRES = []   # (имя, локальные точки до выхода, цвет, целевой y)

# --- SG90
sx, sy = 70, 180
rect(sx, sy + 6, 26, 14, "#1971c2", "#0b3d6b", 1, 3); rect(sx + 176, sy + 6, 26, 14, "#1971c2", "#0b3d6b", 1, 3)
circle(sx + 13, sy + 13, 4, "#14161c"); circle(sx + 189, sy + 13, 4, "#14161c")
rect(sx + 20, sy, 162, 74, "#1c7ed6", "#0b3d6b", 2, 6)
circle(sx + 58, sy + 30, 18, "#f8f9fa", "#adb5bd", 1)
add(f'<path d="M {sx+58-8} {sy+30} L {sx+58-6} {sy-14} Q {sx+58} {sy-22} {sx+58+6} {sy-14} L {sx+58+8} {sy+30} Z" fill="#f8f9fa" stroke="#adb5bd"/>')
circle(sx + 58, sy + 30, 5, "#adb5bd")
text(sx + 125, sy + 36, "SG90", 20, "#f8f9fa", "middle", "bold")
text(sx + 125, sy + 56, "серва 180°", 12, "#d0ebff", "middle")
cy0 = sy + 22
cols = [("#7c4a1e", "GND", "кор."), ("#e03131", "5V", "красн."), ("#ff922b", None, "оранж.")]
for i, (c, net, nm) in enumerate(cols):
    yy = cy0 + i * 22
    wire([(sx + 182, yy), (sx + 236, yy)], c, 4, dot_start=False, dot_end=False)
rect(sx + 236, cy0 - 12, 18, 68, "#212529", "#495057", 1, 2)
for i in range(3):
    pin(sx + 260, cy0 + i * 22, 8)
gnd_x = sx + 300
wire([(sx + 260, cy0), (gnd_x, cy0)], "#7c4a1e", 3, dot_end=False); gnd(gnd_x, cy0)
wire([(sx + 260, cy0 + 22), (sx + 300, cy0 + 22)], NET["5V"], 3, dot_end=False)
tag(sx + 312, cy0 + 22, "5V", "5V", "start")
text(sx + 230, cy0 + 82, "+ 1000 мкФ между 5V и GND у сервы", 12, "#ffa8a8", "start", italic=True)
LEFT_WIRES.append(("servo", [(sx + 260, cy0 + 44), (sx + 372, cy0 + 44)], "#ff922b", ly("4")))

# --- TOF400F
tx, ty = 70, 330
rect(tx, ty, 240, 190, "#1b1d22", "#495057", 2, 18)
for hx, hy in ((tx + 22, ty + 22), (tx + 218, ty + 22), (tx + 22, ty + 168), (tx + 218, ty + 168)):
    circle(hx, hy, 13, "#c9a227"); circle(hx, hy, 8, "#14161c")
add(f'<ellipse cx="{tx+120}" cy="{ty+88}" rx="52" ry="66" fill="#0b0c10" stroke="#343a40" stroke-width="3"/>')
circle(tx + 104, ty + 70, 14, "url(#lens)"); circle(tx + 136, ty + 70, 14, "url(#lens)")
rect(tx + 100, ty + 104, 40, 22, "#212529", "#495057", 1, 3)
text(tx + 120, ty - 10, "TOF400F (VL53L1X)", 15, "#f8f9fa", "middle", "bold")
tof_names = ["TXD", "RXD", "SCL", "SDA", "GND", "VIN"]
tpx = [tx + 45 + i * 30 for i in range(6)]
tpy = ty + 190
for x, n in zip(tpx, tof_names):
    pin(x, tpy + 4, 10)
    text(x + 4, tpy - 8, n, 11, "#f8f9fa", "start", "bold", MONO, rot=-90)
for x in (tpx[2], tpx[3]):
    line(x - 6, tpy + 14, x + 6, tpy + 26, "#fa5252", 2.5); line(x + 6, tpy + 14, x - 6, tpy + 26, "#fa5252", 2.5)
wire([(tpx[4], tpy + 4), (tpx[4], tpy + 18)], NET["GND"], 3, dot_end=False); gnd(tpx[4], tpy + 18)
wire([(tpx[5], tpy + 4), (tpx[5], tpy + 20)], NET["3V3"], 3, dot_end=False); tag(tpx[5] + 10, tpy + 32, "3V3")
text(tx + 250, ty + 40, "крепится на качалку", 13, "#adb5bd", "start", italic=True)
text(tx + 250, ty + 58, "сервы, выше всего", 13, "#adb5bd", "start", italic=True)
text(tx + 250, ty + 76, "на машинке", 13, "#adb5bd", "start", italic=True)
text(tx + 250, ty + 110, "SDA/SCL — НЕ", 13, "#fa5252", "start", "bold")
text(tx + 250, ty + 128, "подключать", 13, "#fa5252", "start", "bold")
LEFT_WIRES.append(("tof_tx", [(tpx[0], tpy + 4), (tpx[0], tpy + 66), (tpx[0] + 320, tpy + 66)], "#51cf66", ly("6")))
LEFT_WIRES.append(("tof_rx", [(tpx[1], tpy + 4), (tpx[1], tpy + 52), (tpx[1] + 290, tpy + 52)], "#f1f3f5", ly("5")))

# --- HC-SR04
def hcsr04(x, y, title, trig_c, echo_c, trig_pin, echo_pin, key, chans):
    rect(x, y, 240, 104, "#1864ab", "#0b3d6b", 2, 6)
    for cx in (x + 58, x + 182):
        circle(cx, y + 50, 38, "url(#ustx)", "#adb5bd", 2)
        circle(cx, y + 50, 26, "#343a40")
        for k in range(4):
            circle(cx, y + 50, 6 + k * 5, "none", "#495057", 1)
    text(x + 120, y + 24, "HC-SR04", 13, "#f8f9fa", "middle", "bold")
    text(x + 120, y - 10, title, 15, "#f8f9fa", "middle", "bold")
    names = ["VCC", "TRIG", "ECHO", "GND"]
    px = [x + 84 + i * 24 for i in range(4)]
    py = y + 104
    for xx, n in zip(px, names):
        pin(xx, py + 4, 10)
        text(xx + 4, py - 6, n, 10, "#f8f9fa", "start", "bold", MONO, rot=-90)
    wire([(px[0], py + 4), (px[0], py + 18)], NET["5V"], 3, dot_end=False); tag(px[0], py + 30, "5V")
    wire([(px[3], py + 4), (px[3], py + 10)], NET["GND"], 3, dot_end=False); gnd(px[3], py + 10)
    # TRIG и ECHO через преобразователь уровней HW-209 (B — 5 В сторона датчика, A — 3.3 В сторона ESP)
    ey, ty = py + 28, py + 48
    bx = x + 290
    wire([(px[2], py + 4), (px[2], ey), (bx, ey)], echo_c, 4, dot_end=False)
    wire([(px[1], py + 4), (px[1], ty), (bx, ty)], trig_c, 4, dot_end=False)
    rect(bx, py + 12, 80, 52, "#0c5c3c", "#2b8a3e", 2, 4)
    text(bx + 40, py + 6, "HW-209 " + chans, 11, "#b2f2bb", "middle", "bold")
    for yy, nb, na in ((ey, "B" + chans[0], "A" + chans[0]), (ty, "B" + chans[2], "A" + chans[2])):
        pin(bx, yy, 8); pin(bx + 80, yy, 8)
        text(bx + 6, yy + 4, nb, 10, "#f8f9fa", "start", "bold", MONO)
        text(bx + 74, yy + 4, na, 10, "#f8f9fa", "end", "bold", MONO)
    LEFT_WIRES.append((key + "_echo", [(bx + 80, ey), (bx + 110, ey)], echo_c, ly(echo_pin)))
    LEFT_WIRES.append((key + "_trig", [(bx + 80, ty), (bx + 110, ty)], trig_c, ly(trig_pin)))

hcsr04(70, 660, "Ультразвук ПЕРЕД", "#ffd43b", "#ff8787", "15", "7", "usf", "1-2")
hcsr04(70, 880, "Ультразвук ЗАД", "#c0eb75", "#f06595", "17", "16", "usr", "3-4")
text(70, 1166, "HW-209 (один модуль): LV/AVCC → 3V3, HV/BVCC → 5V, GND → общая · B — датчики, A — ESP, канал один", 12, "#b2f2bb", italic=True)

# --- ИК пола
ix, iy = 70, 1062
rect(ix, iy, 240, 64, "#1b1d22", "#495057", 2, 6)
circle(ix + 22, iy + 20, 11, "#212529", "#495057"); circle(ix + 22, iy + 46, 11, "#e9ecef", "#adb5bd")
rect(ix + 60, iy + 18, 26, 26, "#1c7ed6", "#0b3d6b", 1, 3); circle(ix + 73, iy + 31, 7, "#f8f9fa")
text(ix + 128, iy + 28, "ИК пола", 14, "#f8f9fa", "middle", "bold")
text(ix + 128, iy + 46, "FC-51/TCRT5000", 10, "#adb5bd", "middle")
ipn = [("OUT", iy + 14), ("GND", iy + 32), ("VCC", iy + 50)]
for n, yy in ipn:
    pin(ix + 240, yy, 9)
    text(ix + 228, yy + 4, n, 10, "#f8f9fa", "end", "bold", MONO)
wire([(ix + 240, iy + 32), (ix + 266, iy + 32)], NET["GND"], 3, dot_end=False); gnd(ix + 266, iy + 32)
wire([(ix + 240, iy + 50), (ix + 286, iy + 50)], NET["3V3"], 3, dot_end=False); tag(ix + 286, iy + 50, "3V3", None, "start")
text(ix, iy + 84, "смотрит в пол, 1–2 см над ним, перед колёсами · подписи сверить с платой", 12, "#fab005", italic=True)
LEFT_WIRES.append(("ir", [(ix + 240, iy + 14), (ix + 360, iy + 14)], "#63e6be", ly("18")))

# --- MX1508
mx, my = 140, 1200
rect(mx, my, 160, 108, "#c92a2a", "#7a1414", 2, 6)
rect(mx + 50, my + 30, 52, 44, "#212529", "#495057", 1, 2)
text(mx + 76, my + 58, "MX1508", 11, "#adb5bd", "middle", "bold")
text(mx + 80, my - 10, "MX1508 — драйвер моторов", 13, "#f8f9fa", "middle", "bold")
ins = ["IN1", "IN2", "IN3", "IN4"]
in_cols = ["#da77f2", "#9775fa", "#f783ac", "#748ffc"]
in_pins = ["10", "11", "8", "9"]   # проверено на машинке: GPIO10/11 → левая сторона
for i, (n, c, p) in enumerate(zip(ins, in_cols, in_pins)):
    yy = my + 16 + i * 22
    pin(mx + 160, yy, 9)
    text(mx + 150, yy + 4, n, 10, "#f8f9fa", "end", "bold", MONO)
    LEFT_WIRES.append(("in" + str(i), [(mx + 160, yy), (mx + 210 + i * 0, yy)], c, ly(p)))
# моторные выходы A — слева, B — снизу справа, питание — снизу слева
pa = [(mx, my + 22), (mx, my + 50)]
for (x, y), n in zip(pa, ("A1", "A2")):
    pin(x, y, 10); text(x + 10, y + 4, n, 10, "#f8f9fa", "start", "bold", MONO)
pb = [(mx + 112, my + 108), (mx + 140, my + 108)]
for (x, y), n in zip(pb, ("B1", "B2")):
    pin(x, y, 10); text(x + 4, y - 8, n, 10, "#f8f9fa", "start", "bold", MONO, rot=-90)
pp = [(mx + 24, my + 108), (mx + 52, my + 108)]
for (x, y), n in zip(pp, ("+", "−")):
    pin(x, y, 10); text(x, y - 10, n, 14, "#f8f9fa", "middle", "bold", MONO)
wire([pp[0], (pp[0][0], pp[0][1] + 18)], NET["7V"], 3, dot_end=False); tag(pp[0][0], pp[0][1] + 30, "7V", "9V")
wire([pp[1], (pp[1][0], pp[1][1] + 10)], NET["GND"], 3, dot_end=False); gnd(pp[1][0], pp[1][1] + 10)

# --- шасси, моторы, энкодеры
CX0, CY0, CX1, CY1 = 120, 1420, 640, 1990
rect(CX0, CY0, CX1 - CX0, CY1 - CY0, "#1a1d24", "#495057", 3, 24)
text((CX0 + CX1) / 2, (CY0 + CY1) / 2 - 10, "ШАССИ (вид сверху)", 18, "#495057", "middle", "bold")
text((CX0 + CX1) / 2, (CY0 + CY1) / 2 + 14, "▲ перед", 16, "#495057", "middle")
MOT_RED, MOT_BLK = "#e03131", "#212529"

def motor(side, front):
    """Возвращает (клеммы мотора [(x,y),(x,y)], пины энкодера dict)."""
    ay = 1470 if front else 1940          # ось колеса
    if side == "L":
        wx, gx0, gx1 = 92, 125, 205
        diskx = 214
        modx = 226
    else:
        wx, gx0, gx1 = 638, 555, 635
        diskx = 546
        modx = 460
    # колесо
    rect(wx, ay - 52, 32, 104, "#212529", "#495057", 2, 8)
    for k in range(9):
        line(wx + 2, ay - 46 + k * 11.5, wx + 30, ay - 46 + k * 11.5, "#343a40", 2)
    # редуктор
    rect(gx0, ay - 22, gx1 - gx0, 44, "#fcc419", "#e67700", 2, 4)
    circle((gx0 + gx1) / 2, ay, 7, "#f8f9fa", "#adb5bd")
    # мотор (банка) — вперёд у передних, назад у задних
    cx0 = gx0 + 10
    if front:
        rect(cx0, ay - 22 - 30, 60, 30, "url(#can)", "#868e96", 1, 6)
        tabs = [(cx0 + 16, ay - 52), (cx0 + 44, ay - 52)]
    else:
        rect(cx0, ay + 22, 60, 30, "url(#can)", "#868e96", 1, 6)
        tabs = [(cx0 + 16, ay + 52), (cx0 + 44, ay + 52)]
    for t in tabs:
        rect(t[0] - 4, t[1] - 4, 8, 8, "#d4af37", "#7a5c00", 1)
    # конденсатор 100 нФ между клеммами
    capy = tabs[0][1] + (-14 if front else 14)
    add(f'<ellipse cx="{(tabs[0][0]+tabs[1][0])/2}" cy="{capy}" rx="9" ry="7" fill="#f08c00" stroke="#a35200"/>')
    line(tabs[0][0], tabs[0][1], (tabs[0][0] + tabs[1][0]) / 2 - 8, capy, "#ced4da", 1.5)
    line(tabs[1][0], tabs[1][1], (tabs[0][0] + tabs[1][0]) / 2 + 8, capy, "#ced4da", 1.5)
    # диск энкодера (на второй оси)
    add(f'<ellipse cx="{diskx}" cy="{ay}" rx="7" ry="30" fill="#212529" stroke="#868e96" stroke-width="1.5"/>')
    for k in range(-4, 5):
        line(diskx - 5, ay + k * 6, diskx + 5, ay + k * 6, "#adb5bd", 1)
    # модуль LM393
    mw, mh = 78, 62
    my_ = ay - 31
    rect(modx, my_, mw, mh, "#1864ab", "#0b3d6b", 2, 4)
    sx_ = modx if side == "L" else modx + mw - 14
    rect(sx_, my_ + 14, 14, 28, "#212529", "#495057", 1)
    text(modx + mw / 2 + (6 if side == "L" else -6), my_ + (22 if front else mh - 10), "LM393", 11, "#d0ebff", "middle", "bold")
    names = ["VCC", "GND", "DO"]
    pxs = [modx + 17 + k * 22 for k in range(3)]
    py_ = my_ + mh if front else my_
    for x, n in zip(pxs, names):
        pin(x, py_, 8)
        text(x, py_ + (-10 if front else 18), n, 9, "#f8f9fa", "middle", "bold", MONO)
    sgn = 1 if front else -1
    wire([(pxs[0], py_), (pxs[0], py_ + 14 * sgn), (pxs[0] - 18, py_ + 14 * sgn)], NET["3V3"], 2.5, dot_end=False)
    tag(pxs[0] - 18, py_ + 14 * sgn, "3V3", None, "end")
    wire([(pxs[1], py_), (pxs[1], py_ + 6 * sgn)], NET["GND"], 2.5, dot_end=False)
    gnd(pxs[1] + 0, py_ + 6 * sgn, down=front)
    text(modx + mw / 2, my_ + (-8 if front else mh + 16) if False else (my_ - 8 if front else my_ + mh + 18),
         {("L", True): "FL", ("R", True): "FR", ("L", False): "RL", ("R", False): "RR"}[(side, front)],
         14, "#f8f9fa", "middle", "bold")
    return tabs, (pxs[2], py_)

tabsFL, doFL = motor("L", True)
tabsFR, doFR = motor("R", True)
tabsRL, doRL = motor("L", False)
tabsRR, doRR = motor("R", False)

# провода моторов: A -> левые (параллельно), B -> правые (параллельно)
A1, A2 = pa
wire([A1, (78, A1[1]), (78, 1392), (tabsFL[1][0], 1392), tabsFL[1]], MOT_RED, 3.5)
wire([A2, (64, A2[1]), (64, 1402), (tabsFL[0][0], 1402), tabsFL[0]], MOT_BLK, 3.5)
wire([(78, 1392), (78, 2010), (tabsRL[1][0], 2010), tabsRL[1]], MOT_RED, 3.5, dot_start=False); junction(78, 1392, MOT_RED)
wire([(64, 1402), (64, 2024), (tabsRL[0][0], 2024), tabsRL[0]], MOT_BLK, 3.5, dot_start=False); junction(64, 1402, "#868e96")
B1, B2 = pb
wire([B1, (B1[0], 1382), (tabsFR[0][0], 1382), tabsFR[0]], MOT_RED, 3.5)
wire([B2, (B2[0], 1372), (tabsFR[1][0], 1372), tabsFR[1]], MOT_BLK, 3.5)
wire([(tabsFR[0][0], 1382), (682, 1382), (682, 2010), (tabsRR[0][0], 2010), tabsRR[0]], MOT_RED, 3.5, dot_start=False)
junction(tabsFR[0][0], 1382, MOT_RED)
wire([(tabsFR[1][0], 1372), (696, 1372), (696, 2024), (tabsRR[1][0], 2024), tabsRR[1]], MOT_BLK, 3.5, dot_start=False)
junction(tabsFR[1][0], 1372, "#868e96")
text(140, 2058, "Моторы одной стороны — параллельно.  На каждом моторе керамика 100 нФ (оранжевая).", 13, "#ffc078")
text(140, 2078, "Направление сторон задаётся в config.h (MOTOR_L/R_INVERT).  AO у энкодеров не подключать.  ШИМ 500 Гц.", 13, "#ffc078")

LEFT_WIRES.append(("encFL", [doFL, (doFL[0], 1546), (doFL[0] + 380, 1546)], "#3bc9db", ly("13")))
wire([doFR, (doFR[0], 1528), (doFR[0] + 40, 1528)], "#38d9a9", 4, dot_end=False)
rect(doFR[0] + 40, 1517, 118, 22, "#101217", "#38d9a9", 2, 11); text(doFR[0] + 99, 1533, "→ ESP GPIO2", 13, "#38d9a9", "middle", "bold")
LEFT_WIRES.append(("encRL", [doRL, (doRL[0], 1838), (doRL[0] + 380, 1838)], "#a5d8ff", ly("14")))
RR_PATH_Y = 1862
enc_rr = [doRR, (doRR[0], RR_PATH_Y)]

# ---------------------------------------------------------------- раскладка дорожек слева
LANES = [760 + i * 24 for i in range(len(LEFT_WIRES))]

def segs(local, lane, ty):
    pts = local + [(lane, local[-1][1]), (lane, ty), (LX, ty)]
    s = []
    for a, b in zip(pts, pts[1:]):
        s.append((a, b))
    return pts, s

def inter(s1, s2):
    (a, b), (c, d) = s1, s2
    h1 = a[1] == b[1]; h2 = c[1] == d[1]
    if h1 == h2:
        return 0
    if not h1:
        (a, b), (c, d) = (c, d), (a, b)
    y = a[1]; x0, x1 = sorted((a[0], b[0])); x = c[0]; y0, y1 = sorted((c[1], d[1]))
    return 1 if (x0 < x < x1 and y0 < y < y1) else 0

def cost(assign):
    ss = [segs(w[1], LANES[assign[i]], w[3])[1] for i, w in enumerate(LEFT_WIRES)]
    n = 0
    for i, j in combinations(range(len(ss)), 2):
        for a in ss[i]:
            for b in ss[j]:
                n += inter(a, b)
    return n

assign = list(range(len(LEFT_WIRES)))
# стартовая раскладка: по целевому пину сверху вниз
order = sorted(range(len(LEFT_WIRES)), key=lambda i: LEFT_WIRES[i][3])
for k, i in enumerate(order):
    assign[i] = k
best = cost(assign)
improved = True
while improved:
    improved = False
    for i, j in combinations(range(len(assign)), 2):
        assign[i], assign[j] = assign[j], assign[i]
        c = cost(assign)
        if c < best:
            best = c; improved = True
        else:
            assign[i], assign[j] = assign[j], assign[i]
print("пересечений слева:", best, file=sys.stderr)

for i, (name, local, color, tyy) in enumerate(LEFT_WIRES):
    pts, _ = segs(local, LANES[assign[i]], tyy)
    # убрать повторяющиеся точки
    clean = [pts[0]]
    for p in pts[1:]:
        if p != clean[-1]:
            clean.append(p)
    wire(clean, color, 4)

# ---------------------------------------------------------------- справа: MPU6050, делитель батареи, ENC RR
gx, gy = 1770, 570
rect(gx, gy, 150, 190, "#5f3dc4", "#36217a", 2, 6)
circle(gx + 128, gy + 22, 9, "#14161c"); circle(gx + 128, gy + 168, 9, "#14161c")
rect(gx + 56, gy + 70, 46, 46, "#212529", "#495057", 1, 3)
text(gx + 79, gy + 98, "MPU", 11, "#adb5bd", "middle", "bold")
text(gx + 79, gy + 150, "GY-521", 14, "#f8f9fa", "middle", "bold")
text(gx + 79, gy + 168, "MPU6050", 11, "#e5dbff", "middle")
mpu = ["VCC", "GND", "SCL", "SDA", "XDA", "XCL", "AD0", "INT"]
mpy = {n: gy + 20 + i * 21 for i, n in enumerate(mpu)}
for n in mpu:
    pin(gx, mpy[n], 9)
    text(gx + 12, mpy[n] + 4, n, 10, "#f8f9fa", "start", "bold", MONO)
for n in ("XDA", "XCL", "AD0", "INT"):
    line(gx - 14, mpy[n] - 5, gx - 4, mpy[n] + 5, "#fa5252", 2); line(gx - 4, mpy[n] - 5, gx - 14, mpy[n] + 5, "#fa5252", 2)
wire([(gx, mpy["VCC"]), (gx - 40, mpy["VCC"])], NET["3V3"], 3, dot_end=False); tag(gx - 40, mpy["VCC"], "3V3", None, "end")
wire([(gx, mpy["GND"]), (gx - 20, mpy["GND"])], NET["GND"], 3, dot_end=False); gnd(gx - 20, mpy["GND"])
wire([(gx, mpy["SCL"]), (1645, mpy["SCL"]), (1645, ry("42")), (RX_, ry("42"))], "#339af0", 4)
wire([(gx, mpy["SDA"]), (1620, mpy["SDA"]), (1620, ry("41")), (RX_, ry("41"))], "#ffe066", 4)
text(gx, gy - 12, "гироскоп: ровно, в центре шасси", 12, "#adb5bd", italic=True)
text(gx, gy + 214, "XDA, XCL, AD0, INT —", 12, "#fa5252", "start", "bold")
text(gx, gy + 230, "не подключать", 12, "#fa5252", "start", "bold")
text(gx, gy + 248, "адрес I2C 0x68", 12, "#adb5bd")

# делитель батареи -> GPIO1
dnx, dny = 1680, ry("1")
wire([(RX_, dny), (dnx, dny)], "#d0bfff", 4, dot_end=False)
junction(dnx, dny, "#d0bfff")
wire([(dnx, dny), (dnx, dny - 24)], "#d0bfff", 3, dot_start=False, dot_end=False)
resistor(dnx, dny - 44, vertical=True, label="100 кОм", bands=("#8b4513", "#000", "#f08c00"))
wire([(dnx, dny - 64), (dnx, dny - 80), (dnx + 30, dny - 80)], NET["12V"], 3, dot_start=False, dot_end=False)
tag(dnx + 30, dny - 80, "12V", "BAT+ после выкл.", "start")
wire([(dnx, dny), (dnx + 60, dny)], "#d0bfff", 3, dot_start=False, dot_end=False)
resistor(dnx + 80, dny, label="", bands=("#e03131", "#7d3cff", "#f08c00"))
text(dnx + 80, dny + 26, "27 кОм", 13, "#f1f3f5", "middle", "bold")
wire([(dnx + 100, dny), (dnx + 116, dny)], "#d0bfff", 3, dot_start=False, dot_end=False)
gnd(dnx + 116, dny)
text(1640, 300, "Батарея (опц., не установлен): 12.6→2.68 В", 13, "#d0bfff", "start", "bold")

# ENC FR — меткой (GPIO12 на этой плате не используется)
wire([(RX_, ry("2")), (RX_ + 44, ry("2"))], "#38d9a9", 4, dot_end=False)
rect(RX_ + 44, ry("2") - 11, 150, 22, "#101217", "#38d9a9", 2, 11); text(RX_ + 119, ry("2") + 5, "← энкодер FR (D0)", 13, "#38d9a9", "middle", "bold")
# ENC RR — под ESP на правую сторону
RR_LANE = 1700
wire(enc_rr + [(RR_LANE, RR_PATH_Y), (RR_LANE, ry("21")), (RX_, ry("21"))], "#96f2d7", 4)

# ---------------------------------------------------------------- блок питания
PX0, PY0 = 1760, 1390
rect(PX0 - 20, PY0 - 50, W - PX0 - 20, 720, "#171a21", "#343a40", 2, 18)
text(PX0, PY0 - 16, "ПИТАНИЕ", 24, "#f8f9fa", weight="bold")
text(PX0 + 130, PY0 - 16, "сначала выставить напряжения понижаек мультиметром, ПОТОМ подключать ESP", 15, "#ff8787", weight="bold")
# BMS
bx_, by_ = PX0 + 330, PY0 + 30
# аккумуляторная сборка 3S с выводами B−, B1, B2, B+
rect(PX0, by_, 270, 176, "#212529", "#495057", 2, 10)
for k in range(3):
    cy_ = by_ + 12 + k * 54
    rect(PX0 + 14, cy_, 210, 44, "#2b8a3e", "#1b5e2a", 2, 20)
    rect(PX0 + 224, cy_ + 13, 8, 18, "#ced4da", "#868e96", 1, 2)
    text(PX0 + 119, cy_ + 28, f"18650 · 3.7 В", 15, "#ebfbee", "middle", "bold")
    text(PX0 + 30, cy_ + 28, "−" if k % 2 == 0 else "+", 16, "#f8f9fa", "middle", "bold")
    text(PX0 + 212, cy_ + 28, "+" if k % 2 == 0 else "−", 16, "#f8f9fa", "middle", "bold")
text(PX0, by_ + 200, "сборка 3S: 12.6 В полный · 11.1 В ном. · 9.0 В пусто", 14, "#b2f2bb", weight="bold")
text(PX0, by_ + 220, "прошивка остановит моторы при ~9.6 В", 13, "#adb5bd", italic=True)
rect(bx_, by_, 120, 176, "#2f9e44", "#1b5e2a", 2, 6)
text(bx_ + 60, by_ + 96, "BMS 3S", 16, "#f8f9fa", "middle", "bold")
bms_l = ["B−", "B1", "B2", "B+"]
tap_c = ["#212529", "#ffd43b", "#ffd43b", NET["12V"]]
tap_t = ["0 В", "3.7 В", "7.4 В", "11.1 В"]
for i, n in enumerate(bms_l):
    yy = by_ + 22 + i * 44
    pin(bx_, yy, 9); text(bx_ + 10, yy + 4, n, 11, "#f8f9fa", "start", "bold", MONO)
    wire([(PX0 + 270, yy), (bx_, yy)], tap_c[i], 3)
    text(PX0 + 300, yy - 8, tap_t[i], 10, "#adb5bd", "middle")
for i, n in enumerate(["P−", "P+"]):
    yy = by_ + 50 + i * 76
    pin(bx_ + 120, yy, 9); text(bx_ + 108, yy + 4, n, 11, "#f8f9fa", "end", "bold", MONO)
# предохранитель + выключатель
P_PLUS = (bx_ + 120, by_ + 126)
P_MINUS = (bx_ + 120, by_ + 50)
fx = bx_ + 170
wire([P_PLUS, (fx, P_PLUS[1])], NET["12V"], 5, dot_end=False)
rect(fx, P_PLUS[1] - 11, 60, 22, "#e9ecef", "#868e96", 1.5, 10); line(fx + 6, P_PLUS[1], fx + 54, P_PLUS[1], "#868e96", 2)
text(fx + 30, P_PLUS[1] - 18, "предохр. 5 А", 12, "#f1f3f5", "middle")
swx = fx + 100
wire([(fx + 60, P_PLUS[1]), (swx, P_PLUS[1])], NET["12V"], 5, dot_start=False, dot_end=False)
rect(swx, P_PLUS[1] - 22, 54, 44, "#212529", "#868e96", 2, 6)
rect(swx + 8, P_PLUS[1] - 16, 38, 32, "#c92a2a", "#7a1414", 1, 4)
text(swx + 27, P_PLUS[1] + 5, "I/O", 12, "#f8f9fa", "middle", "bold")
text(swx + 27, P_PLUS[1] + 40, "выключатель", 12, "#f1f3f5", "middle")
busx = swx + 90
wire([(swx + 54, P_PLUS[1]), (busx, P_PLUS[1])], NET["12V"], 5, dot_start=False, dot_end=False)
junction(busx, P_PLUS[1], NET["12V"])

def buck(x, y, title, vout, sub):
    rect(x, y, 230, 120, "#1971c2", "#0b3d6b", 2, 6)
    circle(x + 150, y + 60, 32, "#343a40", "#212529", 2); circle(x + 150, y + 60, 18, "#c9a227")
    rect(x + 50, y + 18, 28, 28, "#212529", "#495057", 1, 2)
    circle(x + 64, y + 86, 14, "#212529", "#868e96"); text(x + 64, y + 90, "C", 10, "#adb5bd", "middle")
    rect(x + 92, y + 82, 22, 22, "#1c7ed6", "#a5d8ff", 1, 2); circle(x + 103, y + 93, 5, "#e9ecef")
    text(x + 115, y - 10, title, 16, "#f8f9fa", "middle", "bold")
    text(x + 115, y + 146, vout, 17, NET["5V"] if "5" in vout[:2] else NET["7V"], "middle", "bold")
    text(x + 115, y + 166, sub, 12, "#adb5bd", "middle")
    pins_ = {"IN+": (x, y + 34), "IN−": (x, y + 86), "OUT+": (x + 230, y + 34), "OUT−": (x + 230, y + 86)}
    for n, (px_, py_) in pins_.items():
        pin(px_, py_, 10)
        text(px_ + (10 if px_ == x else -10), py_ + 4, n, 10, "#f8f9fa", "start" if px_ == x else "end", "bold", MONO)
    return pins_

b1 = buck(busx + 70, PY0 + 20, "Понижайка 1 (LM2596)", "5.0 В · ≥3 А", "подстроечником, без нагрузки")
b2 = buck(busx + 70, PY0 + 300, "Понижайка 2 (≥3 А)", "9.0 В", "подстроечником, без нагрузки")
wire([(busx, P_PLUS[1]), (busx, b1["IN+"][1]), b1["IN+"]], NET["12V"], 5, dot_start=False)
wire([(busx, P_PLUS[1]), (busx, b2["IN+"][1]), b2["IN+"]], NET["12V"], 5, dot_start=False)
wire([(busx, b2["IN+"][1]), (busx, b2["IN+"][1] + 60), (busx - 20, b2["IN+"][1] + 60)], NET["12V"], 3, dot_start=False, dot_end=False)
tag(busx - 20, b2["IN+"][1] + 60, "12V", "→ делитель GPIO1", "end")

# земля
GY_ = PY0 + 600
wire([P_MINUS, (P_MINUS[0] + 30, P_MINUS[1]), (P_MINUS[0] + 30, GY_), (W - 80, GY_)], "#212529", 5, dot_end=False)
wire([b1["IN−"], (b1["IN−"][0] - 30, b1["IN−"][1]), (b1["IN−"][0] - 30, GY_)], "#212529", 3, dot_end=False)
junction(b1["IN−"][0] - 30, GY_, "#868e96")
wire([b2["IN−"], (b2["IN−"][0] - 30, b2["IN−"][1]), (b2["IN−"][0] - 30, GY_)], "#212529", 3, dot_end=False)
wire([b1["OUT−"], (b1["OUT−"][0] + 20, b1["OUT−"][1]), (b1["OUT−"][0] + 20, GY_)], "#212529", 3, dot_end=False)
junction(b1["OUT−"][0] + 20, GY_, "#868e96")
wire([b2["OUT−"], (b2["OUT−"][0] + 20, b2["OUT−"][1]), (b2["OUT−"][0] + 20, GY_)], "#212529", 3, dot_end=False)
junction(b2["OUT−"][0] + 20, GY_, "#868e96")
gnd(W - 80, GY_)
text(P_MINUS[0] + 50, GY_ - 12, "ОБЩАЯ ЗЕМЛЯ: батарея (P−), обе понижайки, ESP GND, MX1508 −, все датчики, серва", 14, "#ced4da", weight="bold")

# выход 5 В
o1 = b1["OUT+"]
x5 = o1[0] + 70
wire([o1, (x5, o1[1])], NET["5V"], 5, dot_end=False)
junction(x5, o1[1], NET["5V"])
# ветки 5V
wire([(x5, o1[1]), (x5, o1[1] - 30), (x5 + 70, o1[1] - 30)], NET["5V"], 3, dot_start=False, dot_end=False)
tag(x5 + 70, o1[1] - 30, "5V", "→ серва (красный)", "start")
wire([(x5, o1[1]), (x5 + 70, o1[1])], NET["5V"], 3, dot_start=False, dot_end=False)
tag(x5 + 70, o1[1], "5V", "→ HC-SR04 ×2 + HW-209 HV", "start")
# конденсатор сервы
cx_ = x5 + 30
wire([(x5, o1[1]), (x5, o1[1] + 50)], NET["5V"], 3, dot_start=False, dot_end=False)
# диод SS34
dy_ = o1[1] + 50
wire([(x5, dy_), (x5 + 30, dy_)], NET["5V"], 3, dot_start=False, dot_end=False)
add(f'<polygon points="{x5+30},{dy_-12} {x5+30},{dy_+12} {x5+52},{dy_}" fill="#adb5bd" stroke="#f8f9fa" stroke-width="1"/>')
line(x5 + 54, dy_ - 12, x5 + 54, dy_ + 12, "#f8f9fa", 3)
text(x5 + 42, dy_ + 30, "SS34", 12, "#f1f3f5", "middle", "bold")
wire([(x5 + 56, dy_), (x5 + 90, dy_)], NET["5V"], 3, dot_start=False, dot_end=False)
tag(x5 + 90, dy_, "5V", "→ ESP контакт 5V", "start")
text(x5 - 10, dy_ + 50, "нет диода → при подключении USB снимать провод 5V к ESP (GND оставить)", 12, "#ffa8a8", "start", italic=True)
# конденсатор 1000 мкФ
ccx, ccy = x5 + 150, o1[1] + 110
rect(ccx - 16, ccy - 26, 32, 52, "#1c1e3a", "#5c7cfa", 2, 6)
rect(ccx + 6, ccy - 26, 8, 52, "#748ffc", rx=2)
text(ccx + 26, ccy - 6, "1000 мкФ 16 В", 12, "#f1f3f5", "start", "bold")
text(ccx + 26, ccy + 12, "у разъёма сервы: + к 5V, − к GND", 12, "#adb5bd", "start")

# выход 9 В (моторы)
o2 = b2["OUT+"]
wire([o2, (o2[0] + 70, o2[1])], NET["7V"], 5, dot_end=False)
tag(o2[0] + 70, o2[1], "7V", "→ MX1508  +", "start")

# ---------------------------------------------------------------- панель заметок
NX, NY = 1990, 170
rect(NX, NY, W - NX - 30, 1160, "#171a21", "#343a40", 2, 18)
y = NY + 44
text(NX + 30, y, "ВАЖНО", 26, "#ff8787", weight="bold")
notes = [
    ("1", "Понижайки выставить ДО подключения ESP: 5.0 В и 9.0 В (мультиметр)."),
    ("2", "Все GND соединены вместе — без этого ничего не заработает."),
    ("3", "Керамика 100 нФ на выводы КАЖДОГО мотора — иначе помехи и перезагрузки."),
    ("4", "HC-SR04 (5 В) ↔ ESP (3.3 В) — через HW-209: LV=3V3, HV=5V."),
    ("5", "TOF, MPU, энкодеры, ИК — питать от 3V3 ESP (сигналы будут 3.3 В)."),
    ("6", "SS34 перед ESP 5V. Нет диода → при USB снять провод 5V к ESP."),
    ("7", "Конденсатор 1000 мкФ у сервы, иначе ESP сбрасывается при рывках."),
    ("8", "Не занимать GPIO 0, 3, 45, 46, 35–37, 19/20. GPIO12 — свободен."),
    ("9", "TOF400F: SDA и SCL не подключать. MPU: XDA, XCL, AD0, INT не подключать."),
    ("10", "Модули — на разъёмах (гнёзда/Dupont), чтобы можно было снять и проверить."),
    ("11", "Провода моторов — подальше от I2C и энкодеров, пары скрутить."),
]
y += 14
for n, t in notes:
    y += 36
    circle(NX + 46, y - 7, 15, "#c92a2a")
    text(NX + 46, y - 1, n, 15, "#f8f9fa", "middle", "bold")
    text(NX + 74, y, t, 19, "#e9ecef")

y += 54
text(NX + 30, y, "ПАРАМЕТРЫ", 26, "#74c0fc", weight="bold")
params = [
    ("Батарея 3S 18650", "12.6 полн. / 11.1 ном. / 9.0 В пусто"),
    ("Понижайка 1", "5.0 В ≥3 А → ESP (SS34), серва, HC-SR04"),
    ("Понижайка 2", "9.0 В → MX1508 (моторы, предел MX1508 — 10 В)"),
    ("MX1508", "2–10 В · 1.5 А/канал (пик 2.5 А)"),
    ("TT-мотор", "ШИМ 500 Гц · трогается с ~22% · ≤75% от 9 В"),
    ("Колесо 65 мм", "окружность ≈ 204 мм"),
    ("Энкодер, диск 20 щелей", "10.2 мм/щель · 5.1 мм/фронт"),
    ("TOF400F", "3.3 В · UART 115200 Modbus · 4 см…4 м"),
    ("MPU6050 (GY-521)", "3.3 В · I2C 0x68 · гироскоп ±250°/с"),
    ("HC-SR04", "5 В · 2–400 см · Trig 20 мкс · по очереди"),
    ("SG90", "5 В · 50 Гц · импульс 500–2400 мкс"),
    ("Делитель батареи", "опц. 100k/27k: 12.6 В → 2.68 В"),
    ("Нагрузка 3V3 ESP", "≈100–150 мА (запас большой)"),
]
y += 14
for k, v in params:
    y += 32
    rect(NX + 30, y - 23, W - NX - 90, 31, "#1d2129" if params.index((k, v)) % 2 == 0 else "#171a21", rx=4)
    text(NX + 42, y, k, 18, "#a5d8ff", weight="bold")
    text(NX + 400, y, v, 18, "#e9ecef")

y += 52
text(NX + 30, y, "ЦВЕТА", 26, "#ffd43b", weight="bold")
legend = [("12V", "12.6 В батарея"), ("7V", "9 В моторы"), ("5V", "5 В"), ("3V3", "3.3 В"), ("GND", "земля")]
lx = NX + 150
for net, lab in legend:
    line(lx, y - 7, lx + 34, y - 7, NET[net], 6)
    text(lx + 42, y - 1, lab, 14, "#e9ecef")
    lx += 52 + 9 * len(lab)
y += 30
line(NX + 150, y - 7, NX + 184, y - 7, "#e03131", 5); line(NX + 190, y - 7, NX + 224, y - 7, "#212529", 5)
text(NX + 232, y - 1, "провода моторов", 14, "#e9ecef")
text(NX + 400, y - 1, "остальные цвета — сигналы, у каждого свой; подписи на ESP совпадают по цвету", 14, "#adb5bd")

y += 46
text(NX + 30, y, "ПОРЯДОК ПРОВЕРКИ", 22, "#69db7c", weight="bold")
y += 30
text(NX + 30, y, "0 понижайки → 1 ESP от батареи → 2 замер батареи → 3 TOF + серва → 4 MPU → 5 ультразвук", 17, "#e9ecef")
y += 26
text(NX + 30, y, "→ 6 ИК пола → 7 энкодеры (крутить рукой) → 8 моторы (КОЛЁСА В ВОЗДУХЕ!)", 17, "#e9ecef")

add("</svg>")
print("\n".join(out))
