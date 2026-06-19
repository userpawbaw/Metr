#!/usr/bin/env python3
"""Fig 1 v2 - 차동구동 회전 기하 + 마이크로마우스 본체/바퀴 표시.
바퀴가 정확히 어느 호 위에 있는지 보이도록 한다."""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, FancyArrowPatch, Arc, Polygon
import matplotlib.font_manager as fm
import glob, os

OUT = os.path.dirname(os.path.abspath(__file__))

# --- 한글 폰트 등록 (설명은 한글, 기호/변수는 mathtext) ---
for p in glob.glob("/usr/share/fonts/truetype/nanum/NanumGothic.ttf") + \
         glob.glob(os.path.expanduser("~/.fonts/NanumGothic.ttf")):
    try: fm.fontManager.addfont(p)
    except Exception: pass
plt.rcParams.update({"font.size": 12, "font.family": "NanumGothic",
                     "axes.unicode_minus": False})
BLUE, RED, GRAY = "#1f6fb2", "#c0392b", "#8a8a8a"

O = (0.0, 0.0)
R, base = 3.0, 1.3            # inner radius, wheel separation
Rout = R + base
A0, A1 = 16.0, 74.0          # swept angle theta


def arc(cx, cy, r, a0, a1, n=200):
    a = np.linspace(np.radians(a0), np.radians(a1), n)
    return cx + r*np.cos(a), cy + r*np.sin(a)


def units(a):
    ar = np.radians(a)
    rad = np.array([np.cos(ar), np.sin(ar)])      # radial outward (= axle dir)
    head = np.array([-np.sin(ar), np.cos(ar)])    # tangent / heading (CCW)
    return rad, head


def rect(c, along, across, L, W):
    pts = []
    for s1, s2 in [(+1, +1), (+1, -1), (-1, -1), (-1, +1)]:
        pts.append(c + s1*L/2*along + s2*W/2*across)
    return np.array(pts)


def mouse(ax, a, alpha=1.0, label=False):
    rad, head = units(a)
    inner = R * rad
    outer = Rout * rad
    mid = (R + Rout)/2 * rad
    # chassis body (slightly inside the wheels span)
    body = rect(mid, head, rad, L=0.95, W=base*0.92)
    ax.add_patch(Polygon(body, closed=True, facecolor="#d9e6f2",
                         edgecolor="#33536e", lw=1.4, alpha=alpha, zorder=5,
                         joinstyle="round"))
    # two wheels (rolling dir = heading), centered exactly on the arcs
    for c, col in ((inner, RED), (outer, BLUE)):
        w = rect(c, head, rad, L=0.62, W=0.20)
        ax.add_patch(Polygon(w, closed=True, facecolor=col, edgecolor="k",
                             lw=1.0, alpha=alpha, zorder=7))
    # heading arrow
    ax.add_patch(FancyArrowPatch(mid, mid + 0.95*head, arrowstyle="-|>",
                 mutation_scale=15, color="#222", lw=1.8, alpha=alpha, zorder=8))
    if label:
        ax.annotate("안쪽 바퀴\n($s_L$ 호를 따라 이동)", inner,
                    textcoords="offset points", xytext=(-72, -32), color=RED,
                    fontsize=11, ha="center",
                    arrowprops=dict(arrowstyle="->", color=RED, lw=1.2))
        ax.annotate("바깥쪽 바퀴\n($s_R$ 호를 따라 이동)", outer,
                    textcoords="offset points", xytext=(66, 20), color=BLUE,
                    fontsize=11, ha="center",
                    arrowprops=dict(arrowstyle="->", color=BLUE, lw=1.2))


fig, ax = plt.subplots(figsize=(7.6, 7.0))

# filled slice + two wheel-path arcs
ax.add_patch(Wedge(O, Rout, A0, A1, width=base, facecolor="#eef4fb",
                   edgecolor="none", zorder=0))
ax.plot(*arc(*O, R,    A0, A1), color=RED,  lw=3, zorder=2)
ax.plot(*arc(*O, Rout, A0, A1), color=BLUE, lw=3, zorder=2)

# radii (dashed) at start/end
for a in (A0, A1):
    rad, _ = units(a)
    ax.plot([0, Rout*rad[0]], [0, Rout*rad[1]], color=GRAY, lw=1.1, ls="--",
            zorder=1)

# robots: start (faded) + end (solid, labeled)
mouse(ax, A0, alpha=0.45)
mouse(ax, A1, alpha=1.0, label=True)

# theta arc + center
ax.add_patch(Arc(O, 2.1, 2.1, theta1=A0, theta2=A1, color="k", lw=1.5))
am = np.radians((A0+A1)/2)
ax.annotate(r"$\theta$", (1.25*np.cos(am), 1.25*np.sin(am)), fontsize=17,
            ha="center", va="center")
ax.plot(*O, "ko", ms=6)
ax.annotate("O  (회전 중심)", O, textcoords="offset points",
            xytext=(8, -16), ha="left")

# R label on start radius
rad0, _ = units(A0)
ax.annotate(r"$R$", (0.55*R*rad0[0]+0.12, 0.55*R*rad0[1]-0.05), color=GRAY)

# base_L bracket on the start axle (between the two wheels)
rad0, _ = units(A0)
p_in, p_out = R*rad0, Rout*rad0
ax.annotate("", p_out, p_in, arrowprops=dict(arrowstyle="<->", color="k", lw=1.4))
mid0 = (p_in + p_out)/2
ax.annotate(r"$base_L$", (mid0[0]-0.12, mid0[1]), color="k", ha="right",
            fontsize=12)

# arc-name labels along the arcs
xi, yi = arc(*O, R,    (A0+A1)/2+6, (A0+A1)/2+6, 1)
xo, yo = arc(*O, Rout, (A0+A1)/2-6, (A0+A1)/2-6, 1)
ax.annotate(r"$s_L$ (안쪽 호)", (xi[0], yi[0]), color=RED, fontsize=12,
            ha="center")
ax.annotate(r"$s_R$ (바깥쪽 호)", (xo[0], yo[0]), color=BLUE, fontsize=12,
            ha="center")

ax.set_aspect("equal"); ax.axis("off")
ax.set_title("차동구동 회전 — 두 바퀴가 같은 중심각 " r"$\theta$" "를\n"
             "공유하는 두 호를 그린다", fontsize=13)
ax.set_xlim(-1.3, Rout+0.7); ax.set_ylim(-0.9, Rout+0.8)
fig.tight_layout(); fig.savefig(f"{OUT}/fig1_arc_model_v2.png", dpi=150)
print("saved fig1_arc_model_v2.png")
