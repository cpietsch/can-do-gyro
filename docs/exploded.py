# Generates docs/exploded.svg (python3 docs/exploded.py, run from the repo root).
# Isometric, to scale in mm: Unit RollerCAN 40 x 40 x 40 (motor Ø41), LiPo 71 x 15.5 x 13.5, XIAO 21 x 17.8.
# Axes: z up (the motor axis), +x towards the RollerCAN's Grove port (PORT.A), +y towards its XT30 (2+2) port.
import math

C30 = math.cos(math.pi / 6)
view = {"s": 5.0, "cx": 500, "cy": 830}  # px per mm, screen position of (0, 0, 0)
out = []


def P(x, y, z):
    s = view["s"]
    return (view["cx"] + (x - y) * C30 * s, view["cy"] + (x + y) * 0.5 * s - z * s)


def pts(ps):
    return " ".join(f"{x:.1f},{y:.1f}" for x, y in ps)


def poly(ps, fill, stroke="#1a1a1a", sw=1.3):
    out.append(f'<polygon points="{pts(ps)}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" stroke-linejoin="round"/>')


def box(x0, x1, y0, y1, z0, z1, top, xface, yface, sw=1.3):
    # visible faces only: +x (right), +y (left), top
    poly([P(x1, y0, z0), P(x1, y1, z0), P(x1, y1, z1), P(x1, y0, z1)], xface, sw=sw)
    poly([P(x0, y1, z0), P(x1, y1, z0), P(x1, y1, z1), P(x0, y1, z1)], yface, sw=sw)
    poly([P(x0, y0, z1), P(x1, y0, z1), P(x1, y1, z1), P(x0, y1, z1)], top, sw=sw)


def xrect(x, y0, y1, z0, z1, fill, sw=1):  # a rectangle on a +x face
    poly([P(x, y0, z0), P(x, y1, z0), P(x, y1, z1), P(x, y0, z1)], fill, sw=sw)


def yrect(y, x0, x1, z0, z1, fill, sw=1):  # a rectangle on a +y face
    poly([P(x0, y, z0), P(x1, y, z0), P(x1, y, z1), P(x0, y, z1)], fill, sw=sw)


def cyl(r, z0, z1, side):
    s = view["s"]
    rx, ry = r * s * math.sqrt(2) * C30, r * s * math.sqrt(2) * 0.5
    xc, yb = P(0, 0, z0)
    _, yt = P(0, 0, z1)
    out.append(f'<path d="M{xc-rx:.1f},{yt:.1f} L{xc-rx:.1f},{yb:.1f} A{rx:.1f},{ry:.1f} 0 0 0 {xc+rx:.1f},{yb:.1f} '
               f'L{xc+rx:.1f},{yt:.1f} A{rx:.1f},{ry:.1f} 0 0 1 {xc-rx:.1f},{yt:.1f} Z" fill="{side}" stroke="#1a1a1a" stroke-width="1.3"/>')


def disc(x, y, z, r, fill, sw=0.8):  # a small circle lying flat
    s = view["s"]
    cx, cy = P(x, y, z)
    out.append(f'<ellipse cx="{cx:.1f}" cy="{cy:.1f}" rx="{r*s*math.sqrt(2)*C30:.1f}" ry="{r*s*math.sqrt(2)*0.5:.1f}" '
               f'fill="{fill}" stroke="#1a1a1a" stroke-width="{sw}"/>')


def text(x, y, s, size=14, weight="normal", anchor="start", fill="#222", transform=""):
    t = f' transform="{transform}"' if transform else ""
    out.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" font-weight="{weight}" text-anchor="{anchor}" fill="{fill}"{t}>{s}</text>')


def wire(ps3, color, w=2.4):
    # cubic Bezier through 3D control points (the projection is affine, so it stays a Bezier)
    a, b, c, d = [P(*p) for p in ps3]
    path = f"M{a[0]:.1f},{a[1]:.1f} C{b[0]:.1f},{b[1]:.1f} {c[0]:.1f},{c[1]:.1f} {d[0]:.1f},{d[1]:.1f}"
    if color == "white":
        out.append(f'<path d="{path}" fill="none" stroke="#8a8a8a" stroke-width="{w+1.8}" stroke-linecap="round"/>')
        color = "#fafafa"
    else:  # a light edge, so dark wires show on dark parts
        out.append(f'<path d="{path}" fill="none" stroke="#e8e8e8" stroke-width="{w+1.6}" stroke-linecap="round"/>')
    out.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="{w}" stroke-linecap="round"/>')


def line(p, q, color="#999", w=1, dash=""):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" y2="{q[1]:.1f}" stroke="{color}" stroke-width="{w}"{d}/>')


def callout(p, tx, ty, s, anchor="start", size=12):
    out.append(f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="2.2" fill="#999"/>')
    line(p, (tx + (-4 if anchor == "start" else 4), ty - 4))
    text(tx, ty, s, size, anchor=anchor, fill="#555")


# ---- the XIAO with its two glued-on DuPont sockets
XX, XY = 8.9, 10.5   # half of 17.8 x 21
T_PCB = 1.2
SOCK = 2.5           # socket height


def xiao(z0, grove_wires=None):
    """Draws the XIAO at height z0 (USB-C towards +y). grove_wires() is called before the board is drawn: the
    5V/GND cable leaves the socket's back end, so the board hides its start."""
    z1 = z0 + T_PCB
    zh = z1 + SOCK / 2
    if grove_wires:
        grove_wires(zh)
    box(-XX, XX, -XY, XY, z0, z1, "#2a2a2a", "#151515", "#101010")
    for i in range(7):  # castellated pads
        py = -7.62 + 2.54 * i
        poly([P(XX - 1.3, py - 0.8, z1), P(XX, py - 0.8, z1), P(XX, py + 0.8, z1), P(XX - 1.3, py + 0.8, z1)], "#d4a017", sw=0.4)
        poly([P(-XX, py - 0.8, z1), P(-XX + 1.3, py - 0.8, z1), P(-XX + 1.3, py + 0.8, z1), P(-XX, py + 0.8, z1)], "#d4a017", sw=0.4)
    box(-4, 4, -8.5, 4, z1, z1 + 0.8, "#eeeeee", "#cfcfcf", "#bdbdbd", sw=0.8)  # label area
    # CAN socket's pigtails: from its back end over the board's -y edge to the CANH/CANL pads on the back
    wire([(-7.9, -5, zh), (-7.9, -10, zh + 1), (-5.5, -12, z1), (-5.5, -11, z0 - 0.5)], "white", 1.6)
    wire([(-5.9, -5, zh), (-5.9, -9, zh + 1), (-3.5, -12, z1), (-3.5, -11, z0 - 0.5)], "#16a085", 1.6)
    box(-9.4, -4.4, -5, 9, z1, z1 + SOCK, "#2e2e2e", "#1c1c1c", "#141414", sw=1)  # CAN socket, opening +y
    for px in (-8.6, -6.1):
        yrect(9, px, px + 1.2, z1 + 0.6, z1 + 1.9, "#000", sw=0.4)
    box(-4.5, 4.5, 7, 11.3, z1, z1 + 3.2, "#cfcfcf", "#a9a9a9", "#929292", sw=1)  # USB-C
    box(4.4, 9.4, -6, 8, z1, z1 + SOCK, "#2e2e2e", "#1c1c1c", "#141414", sw=1)  # 5V/GND socket, opening -y
    # its pigtails: from its front end down to the 5V (VBUS) and GND pads next to the USB-C
    wire([(5.9, 8, zh), (5.9, 10.5, zh + 0.5), (8.4, 10, z1 + 0.2), (8.4, 7.62, z1)], "#d63031", 1.6)
    wire([(7.9, 8, zh), (7.9, 10, zh + 0.5), (9.6, 7, z1 + 0.2), (8.4, 5.08, z1)], "#222", 1.6)
    return z1, zh


# ======== main view ========
Z_HOUSING = 28           # RollerCAN: motor 0..28, square housing 28..40
Z_BATT = 40 + 22         # parts pulled apart along the motor axis
Z_XIAO = Z_BATT + 13.5 + 18
BX, BY = 7.75, 35.5      # half of the LiPo's 15.5 x 71

# motor axis, behind everything
line(P(0, 0, Z_XIAO + 16), P(0, 0, -8), "#aaa", 1.2, "8 5 2 5")
text(P(0, 0, Z_XIAO + 16)[0] + 6, P(0, 0, Z_XIAO + 16)[1] + 4, "motor axis", 12, fill="#999")

# Unit RollerCAN, upside down: the motor's rotor (with the mounting plate) stands on the table
cyl(20.0, 0, 8, "#3b3b3b")           # mounting plate, turns with the rotor
cyl(20.5, 8, 21, "#303030")          # rotor bell (D3504, Ø41), turns
cyl(19.5, 21, Z_HOUSING, "#262626")  # stator mount, fixed to the housing
th = math.radians(62)                # the rotor's own Grove port (unused)
t = (-math.sin(th), math.cos(th))
c = (20.0 * math.cos(th), 20.0 * math.sin(th))
poly([P(c[0] - t[0] * 4, c[1] - t[1] * 4, 1.5), P(c[0] + t[0] * 4, c[1] + t[1] * 4, 1.5),
      P(c[0] + t[0] * 4, c[1] + t[1] * 4, 6.5), P(c[0] - t[0] * 4, c[1] - t[1] * 4, 6.5)], "#c0392b", sw=1)
box(-20, 20, -20, 20, Z_HOUSING, 40, "#4a4a4a", "#363636", "#2a2a2a")  # housing 40 x 40
poly([P(-7, -8, 40), P(7, -8, 40), P(7, 6, 40), P(-7, 6, 40)], "#14304a", sw=1)  # 0.66" OLED
for sx, sy in ((-16, -16), (16, -16), (16, 16), (-16, 16)):
    disc(sx, sy, 40, 1.8, "#9a9a9a")
xrect(20, -13, -3, 31, 37, "#c0392b")             # PORT.A (Grove) on the +x face
xrect(20, -12, -4, 32, 35.5, "#5a1a14", sw=0.6)
yrect(20, -1, 9, 30, 38, "#111")                  # XT30 (2+2) on the +y face
yrect(20, 0, 8, 34.5, 37.2, "#000", sw=0.6)
yrect(20, 0, 8, 30.8, 33.5, "#000", sw=0.6)

# plugs, pulled out of their ports
box(-1, 9, 27, 37, 30, 38, "#2c2c2c", "#1f1f1f", "#161616")  # XT30 (2+2) plug
line(P(4, 20.5, 34), P(4, 26.5, 34), "#888", 1, "3 3")
box(26, 33, -13, -3, 31, 37, "#f6f6f6", "#e2e2e2", "#d4d4d4")  # Grove plug
line(P(20.5, -8, 34), P(25.5, -8, 34), "#888", 1, "3 3")

# LiPo, lying along y across the housing
box(-BX, BX, -BY, BY, Z_BATT, Z_BATT + 13.5, "#b9c0c6", "#9aa2a9", "#7e868d")
xrect(BX, -22, 16, Z_BATT + 2.5, Z_BATT + 11, "#ffffff", sw=0.8)
lp = P(BX, 13, Z_BATT + 5)
text(0, 0, "2S LiHV 580 mAh", 15, "bold", fill="#333", transform=f"matrix({C30:.4f},-0.5,0,1,{lp[0]:.1f},{lp[1]:.1f})")
yrect(BY, -4.5, 4.5, Z_BATT + 3, Z_BATT + 10, "#222", sw=0.8)  # heat shrink where the leads leave


def grove_main(zh):  # 5V/GND: the XIAO's 5V socket -> Grove plug
    wire([(5.9, -6, zh), (5.9, -20, zh + 6), (52, -14, 56), (33, -9.5, 34)], "#d63031")
    wire([(7.9, -6, zh), (7.9, -22, zh + 6), (54, -10, 54), (33, -6.5, 34)], "#222")


z1, ZH = xiao(Z_XIAO, grove_main)

# CAN_H / CAN_L: the XIAO's CAN socket -> XT30 plug (lower pins)
wire([(-7.9, 9, ZH), (-7.9, 30, ZH - 4), (0, 60, 50), (6, 37, 32)], "white")
wire([(-5.9, 9, ZH), (-5.9, 28, ZH - 4), (3, 56, 50), (2, 37, 32)], "#16a085")
# LiPo + / -: battery -> XT30 plug (upper pins)
wire([(2, BY, Z_BATT + 6.5), (2, 46, Z_BATT + 4), (6, 46, 40), (6, 37, 36)], "#d63031", 3.4)
wire([(-2, BY, Z_BATT + 6.5), (-2, 43, Z_BATT + 4), (2, 43, 40), (2, 37, 36)], "#222", 3.4)

# the top's front
a0, a1 = P(9.8, 1, z1 + 1.5), P(26, 1, z1 + 1.5)
out.append(f'<line x1="{a0[0]:.1f}" y1="{a0[1]:.1f}" x2="{a1[0]:.1f}" y2="{a1[1]:.1f}" stroke="#2e86de" stroke-width="3" marker-end="url(#arrowBlue)"/>')

# rotation arrow around the turning part
xc, yc = P(0, 0, 5)
rx, ry = 20.5 * 5 * math.sqrt(2) * C30 + 20, 20.5 * 5 * math.sqrt(2) * 0.5 + 12
out.append(f'<path d="M{xc+rx*math.cos(math.radians(20)):.1f},{yc+ry*math.sin(math.radians(20)):.1f} '
           f'A{rx:.1f},{ry:.1f} 0 0 1 {xc+rx*math.cos(math.radians(160)):.1f},{yc+ry*math.sin(math.radians(160)):.1f}" '
           f'fill="none" stroke="#e67e22" stroke-width="3" marker-end="url(#arrowOrange)"/>')
text(xc, yc + ry + 24, "the bottom turns (by hand, or the motor against the top)", 13, anchor="middle", fill="#e67e22")

# labels
LX = 750


def label(p, y, title, lines, lx=LX):
    out.append(f'<polyline points="{p[0]:.1f},{p[1]:.1f} {lx-10:.1f},{y:.1f} {lx-3:.1f},{y:.1f}" fill="none" stroke="#999" stroke-width="1"/>')
    out.append(f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="2.5" fill="#999"/>')
    text(lx, y + 5, title, 15, "bold")
    for i, l in enumerate(lines):
        text(lx, y + 23 + 17 * i, l, 13, fill="#555")


label(P(-2, -10.5, z1 + 0.4), 170, "XIAO STM32C5", ["taped on the LiPo; IMU (LSM6DS3TR-C) on",
      "board, CAN via the on-board TCAN3403;", "two DuPont sockets glued on top (see left)"])
label(P(BX, -30, Z_BATT + 13.5), 300, "2S LiHV LiPo", ["580 mAh, 71 × 15.5 × 13.5 mm, taped",
      "across the RollerCAN; its + / − leads go", "to the XT30 (2+2) plug"])
label(P(20, -18, 39), 470, "Unit RollerCAN", ["40 × 40 × 40 mm: housing with OLED and",
      "FOC driver; Grove PORT.A (5V for the XIAO),", "XT30 (2+2): battery and CAN_H / CAN_L"])
label(P(20.5 * 0.7, -20.5 * 0.7, 12), 620, "Motor (D3504, Ø41)", ["upside down: its rotor stands on the",
      "table and turns, the housing holds still"])
pp = P(4, 37, 31)
out.append(f'<circle cx="{pp[0]:.1f}" cy="{pp[1]:.1f}" r="2.5" fill="#999"/>')
line(pp, (300, 806))
text(296, 800, "XT30 (2+2) plug", 15, "bold", anchor="end")
text(296, 818, "battery + / −,", 13, anchor="end", fill="#555")
text(296, 835, "CAN_H / CAN_L", 13, anchor="end", fill="#555")

# legend
ly0 = 760
items = ((("#16a085", "white"), "CAN_H / CAN_L: XIAO CAN socket → XT30 plug", 2.4),
         (("#d63031", "#222"), "5V / GND: Grove plug → XIAO 5V socket", 2.4),
         (("#d63031", "#222"), "LiPo + / −: battery → XT30 plug", 3.4))
for i, (cs, t_, w) in enumerate(items):
    y = ly0 + 22 * i
    for j, c_ in enumerate(cs):
        yy = y - 3 + 6 * j
        if c_ == "white":
            line((LX, yy), (LX + 28, yy), "#8a8a8a", w + 1.8)
            c_ = "#fafafa"
        line((LX, yy), (LX + 28, yy), c_, w)
    text(LX + 38, y + 4, t_, 12, fill="#555")
y = ly0 + 22 * 3
out.append(f'<line x1="{LX}" y1="{y}" x2="{LX+24}" y2="{y}" stroke="#2e86de" stroke-width="3" marker-end="url(#arrowBlue)"/>')
text(LX + 38, y + 4, "the top's front: keeps facing one direction", 12, fill="#555")

# ======== inset: the XIAO enlarged ========
view.update(s=9.0, cx=318, cy=232)
text(24, 100, "XIAO STM32C5, enlarged", 15, "bold")
text(24, 118, "21 × 17.8 mm, seen from the same side", 12, fill="#777")


def grove_inset(zh):
    wire([(5.9, -6, zh), (5.9, -12, zh + 0.5), (5.9, -15, zh + 1), (5.9, -18, zh + 1)], "#d63031")
    wire([(7.9, -6, zh), (7.9, -12, zh + 0.5), (7.9, -15, zh + 1), (7.9, -18, zh + 1)], "#222")


z1i, zhi = xiao(0, grove_inset)
wire([(-7.9, 9, zhi), (-7.9, 13, zhi - 0.3), (-7.9, 16, zhi - 0.6), (-7.9, 19, zhi - 0.8)], "white")
wire([(-5.9, 9, zhi), (-5.9, 13, zhi - 0.3), (-5.9, 16, zhi - 0.6), (-5.9, 19, zhi - 0.8)], "#16a085")
e = P(-6.9, 19.5, zhi - 0.8)
text(e[0] - 6, e[1] + 22, "to the XT30 plug", 12, anchor="middle", fill="#555")
e = P(6.9, -18.5, zhi + 1)
text(e[0] + 10, e[1] + 8, "to the Grove port", 12, fill="#555")
callout(P(-4.5, -11.2, -0.2), 390, 136, "CANH / CANL pads (back)")
callout(P(8.6, 6.3, z1i), 386, 318, "5V / GND pads")
callout(P(0, 11.3, z1i + 1.6), 200, 318, "USB-C", anchor="end")
callout(P(-6.9, 2, z1i + SOCK), 110, 166, "CAN socket", anchor="end")
callout(P(6.9, 4, z1i + SOCK), 486, 252, "5V / GND socket")

W, H = 1110, 940
svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="Helvetica, Arial, sans-serif">
<defs>
<marker id="arrowBlue" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="#2e86de"/></marker>
<marker id="arrowOrange" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="#e67e22"/></marker>
</defs>
<rect width="100%" height="100%" fill="#ffffff"/>
<text x="24" y="40" font-size="22" font-weight="bold" fill="#222">can-do-gyro, exploded</text>
<text x="24" y="62" font-size="13" fill="#666">to scale; the top (housing, LiPo, XIAO) holds its direction, the bottom turns</text>
{chr(10).join(out)}
</svg>
'''
open("docs/exploded.svg", "w").write(svg)
