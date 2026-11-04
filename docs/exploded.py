# Generates docs/exploded.svg (python3 docs/exploded.py, run from the repo root)
import math
S, CX, CY = 4.0, 290, 770
C30 = math.cos(math.pi / 6)
out = []

def P(x, y, z):
    return (CX + (x - y) * C30 * S, CY + (x + y) * 0.5 * S - z * S)

def poly(pts, fill, stroke="#222", sw=1.6, extra=""):
    d = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
    out.append(f'<polygon points="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" stroke-linejoin="round" {extra}/>')

def box(w, d, zb, h, top, right, left, ox=0, oy=0):
    x0, x1, y0, y1, zt = ox - w / 2, ox + w / 2, oy - d / 2, oy + d / 2, zb + h
    poly([P(x1, y0, zb), P(x1, y1, zb), P(x1, y1, zt), P(x1, y0, zt)], right)
    poly([P(x0, y1, zb), P(x1, y1, zb), P(x1, y1, zt), P(x0, y1, zt)], left)
    poly([P(x0, y0, zt), P(x1, y0, zt), P(x1, y1, zt), P(x0, y1, zt)], top)

def cyl(r, zb, h, top, side):
    rx, ry = r * S * math.sqrt(2) * C30, r * S * math.sqrt(2) * 0.5
    xc, yb = P(0, 0, zb)
    _, yt = P(0, 0, zb + h)
    out.append(f'<path d="M{xc-rx:.1f},{yt:.1f} L{xc-rx:.1f},{yb:.1f} A{rx:.1f},{ry:.1f} 0 0 0 {xc+rx:.1f},{yb:.1f} '
               f'L{xc+rx:.1f},{yt:.1f} A{rx:.1f},{ry:.1f} 0 0 1 {xc-rx:.1f},{yt:.1f} Z" fill="{side}" stroke="#222" stroke-width="1.6"/>')
    out.append(f'<ellipse cx="{xc:.1f}" cy="{yt:.1f}" rx="{rx:.1f}" ry="{ry:.1f}" fill="{top}" stroke="#222" stroke-width="1.6"/>')
    return xc, yt, rx, ry

def text(x, y, s, size=15, weight="normal", anchor="start", fill="#222"):
    out.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" font-weight="{weight}" text-anchor="{anchor}" fill="{fill}">{s}</text>')

def leader(p, lx, ly):
    out.append(f'<polyline points="{p[0]:.1f},{p[1]:.1f} {lx-8:.1f},{ly:.1f} {lx-2:.1f},{ly:.1f}" fill="none" stroke="#888" stroke-width="1"/>')
    out.append(f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="2.5" fill="#888"/>')

def wire(pts, color, dash=""):
    (x0, y0), (x1, y1), (x2, y2), (x3, y3) = pts
    out.append(f'<path d="M{x0:.1f},{y0:.1f} C{x1:.1f},{y1:.1f} {x2:.1f},{y2:.1f} {x3:.1f},{y3:.1f}" fill="none" '
               f'stroke="{color}" ' + ('' if 'stroke-width' in dash else 'stroke-width="2.6" ') + f'stroke-linecap="round" {dash}/>')

G1, G2, G3 = 24, 34, 30
Z_STATOR = 9 + G1
Z_HOUSING = Z_STATOR + 12
Z_BATT = Z_HOUSING + 15 + G2
Z_XIAO = Z_BATT + 14 + G3

# motor axis (behind everything)
xa, ytop = P(0, 0, Z_XIAO + 24)
_, ybot = P(0, 0, -10)
out.append(f'<line x1="{xa}" y1="{ytop}" x2="{xa}" y2="{ybot}" stroke="#999" stroke-width="1.2" stroke-dasharray="8 5 2 5"/>')
text(xa + 6, ytop + 4, "motor axis", 12, fill="#888")

# rotor (bottom) with feet
cyl(20, 0, 9, "#4a4a4a", "#2e2e2e")
for ang in (200, 250, 300):
    a = math.radians(ang)
    x, y = P(19.5 * math.cos(a) * 1.0, 19.5 * math.sin(a), 0)
for fx, fy in ((14, 10), (-4, 19), (17, -6)):
    x, y = P(fx, fy, 0)
    out.append(f'<rect x="{x-5:.1f}" y="{y-2:.1f}" width="10" height="7" rx="2" fill="#c0392b" stroke="#222" stroke-width="1"/>')

# RollerCAN body: stator + housing
cyl(20, Z_STATOR, 12, "#3a3a3a", "#2b2b2b")
box(42, 42, Z_HOUSING, 15, "#555", "#333", "#222")
# screen on top of the housing
poly([P(-12, -12, Z_HOUSING + 15), P(12, -12, Z_HOUSING + 15), P(12, 12, Z_HOUSING + 15), P(-12, 12, Z_HOUSING + 15)], "#1d3b57", sw=1.2)
# Grove port on the left-front face (y = +21), XT30 on the right-front face (x = +21)
gx, gy = P(8, 21, Z_HOUSING + 5)
out.append(f'<rect x="{gx-10:.1f}" y="{gy-7:.1f}" width="20" height="11" fill="#eee" stroke="#222" stroke-width="1.2" transform="skewY(30 {gx:.1f} {gy:.1f})"/>')
xx, xy = P(21, -2, Z_HOUSING + 5)
out.append(f'<rect x="{xx-11:.1f}" y="{xy-7:.1f}" width="22" height="12" rx="2" fill="#e8b400" stroke="#222" stroke-width="1.2" transform="skewY(-30 {xx:.1f} {xy:.1f})"/>')

# battery
box(62, 31, Z_BATT, 14, "#5a6470", "#3d454e", "#2f363d")
lx, ly = P(14, 15.5, Z_BATT + 7)
poly([P(4, 15.5, Z_BATT + 3), P(24, 15.5, Z_BATT + 3), P(24, 15.5, Z_BATT + 11), P(4, 15.5, Z_BATT + 11)], "#f4f4f4", sw=1)
text(lx, ly + 3, "2S LiHV", 9, anchor="middle")

# XIAO STM32C5 (board + USB-C + shield can)
box(21, 17.8, Z_XIAO, 1.6, "#3b3b8f", "#2a2a66", "#20204f")
box(9, 7.5, Z_XIAO + 1.6, 3.2, "#cfcfcf", "#a8a8a8", "#8e8e8e", ox=-1, oy=0)
box(7, 9, Z_XIAO + 1.6, 3.2, "#d9d9d9", "#b0b0b0", "#999", ox=-11, oy=0)
# front arrow on the XIAO
ax0, ay0 = P(4, 0, Z_XIAO + 6)
ax1, ay1 = P(28, 0, Z_XIAO + 6)
out.append(f'<line x1="{ax0:.1f}" y1="{ay0:.1f}" x2="{ax1:.1f}" y2="{ay1:.1f}" stroke="#2e86de" stroke-width="3" marker-end="url(#arrowBlue)"/>')
text(ax1 + 6, ay1 + 14, "front: keeps facing", 12, fill="#2e86de")
text(ax1 + 6, ay1 + 28, "one direction", 12, fill="#2e86de")

# wires: CAN_H/CAN_L/GND XIAO -> XT30, Grove 5V/GND -> XIAO 5V, battery -> XT30
p_xiao_r = P(10.5, 6, Z_XIAO + 0.8)
p_xiao_l = P(-6, 8.9, Z_XIAO + 0.8)
wire([p_xiao_r, (p_xiao_r[0] + 90, p_xiao_r[1] + 30), (xx + 70, xy - 60), (xx + 8, xy - 2)], "#16a085")
white = [(p_xiao_r[0], p_xiao_r[1] + 3), (p_xiao_r[0] + 80, p_xiao_r[1] + 45), (xx + 60, xy - 40), (xx + 4, xy + 3)]
wire(white, "#999", 'stroke-width="4.4"')
wire(white, "#f7f7f7")
wire([p_xiao_l, (p_xiao_l[0] - 110, p_xiao_l[1] + 40), (gx - 90, gy - 70), (gx - 6, gy - 1)], "#d63031")
wire([(p_xiao_l[0] + 3, p_xiao_l[1] + 3), (p_xiao_l[0] - 100, p_xiao_l[1] + 50), (gx - 80, gy - 60), (gx + 2, gy + 2)], "#222")
pb = P(31, 6, Z_BATT + 7)
wire([pb, (pb[0] + 50, pb[1] + 10), (xx + 50, xy - 25), (xx + 10, xy + 2)], "#c0392b", 'stroke-dasharray="6 4"')

# rotation arrow around the rotor
xc, yc = P(0, 0, 4.5)
rx, ry = 20 * S * math.sqrt(2) * C30 + 22, 20 * S * math.sqrt(2) * 0.5 + 14
out.append(f'<path d="M{xc+rx*math.cos(math.radians(20)):.1f},{yc+ry*math.sin(math.radians(20)):.1f} '
           f'A{rx:.1f},{ry:.1f} 0 0 1 {xc+rx*math.cos(math.radians(160)):.1f},{yc+ry*math.sin(math.radians(160)):.1f}" '
           f'fill="none" stroke="#e67e22" stroke-width="3" marker-end="url(#arrowOrange)"/>')
text(xc, yc + ry + 26, "the bottom turns (by hand, or the motor against the top)", 13, anchor="middle", fill="#e67e22")

# labels
LX = 560
def label(p, y, title, lines):
    leader(p, LX, y)
    text(LX, y + 5, title, 15, "bold")
    for i, l in enumerate(lines):
        text(LX, y + 23 + 17 * i, l, 13, fill="#555")

label(P(0, -9, Z_XIAO + 1.6), P(0, -9, Z_XIAO)[1] - 40, "XIAO STM32C5", ["on-board IMU (LSM6DS3TR-C) tracks the", "top's 3D orientation; CAN via the", "on-board TCAN3403; taped on the battery"])
label(P(31, -15.5, Z_BATT + 10), P(31, -15.5, Z_BATT + 10)[1] - 20, "2S LiHV battery", ["580 mAh, 7.6 V, on the XT30;", "lies flat on the RollerCAN's body"])
label(P(21, -21, Z_HOUSING + 12), P(21, -21, Z_HOUSING + 12)[1] - 10, "Unit RollerCAN body", ["motor stator, FOC driver, encoder, screen;", "XT30 (2+2): power + CAN_H/CAN_L,", "Grove: 5 V for the XIAO"])
label(P(20, -10, 4), P(20, -10, 4)[1] - 10, "Rotor = the bottom", ["motor rotor (D3504 BLDC), stands on the", "table on rubber feet"])

# wire legend
ly0 = 830
for i, (c, t, dash) in enumerate((("#16a085", "CAN_H / CAN_L (+ GND): XIAO pads to XT30", ""),
                                  ("#d63031", "5 V / GND: Grove port to XIAO 5V", ""),
                                  ("#c0392b", "battery to XT30 power", 'stroke-dasharray="6 4"'))):
    y = ly0 + 20 * i
    out.append(f'<line x1="{LX}" y1="{y}" x2="{LX+26}" y2="{y}" stroke="{c}" stroke-width="2.6" {dash}/>')
    text(LX + 34, y + 4, t, 12, fill="#555")

W, H = 960, 920
svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="Helvetica, Arial, sans-serif">
<defs>
<marker id="arrowBlue" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="#2e86de"/></marker>
<marker id="arrowOrange" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="#e67e22"/></marker>
</defs>
<rect width="100%" height="100%" fill="#ffffff"/>
<text x="24" y="40" font-size="22" font-weight="bold" fill="#222">can-do-gyro, exploded</text>
<text x="24" y="62" font-size="13" fill="#666">top (body + battery + XIAO) holds its direction, the bottom turns</text>
{chr(10).join(out)}
</svg>
'''
open("/home/dev/can-do-gyro/docs/exploded.svg", "w").write(svg)
