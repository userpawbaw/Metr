#!/usr/bin/env python3
"""차동구동 회전 기하 설명용 그림 생성 (보고서용).
labels: math/English (Korean fonts unavailable). Outputs PNGs in this dir.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, FancyArrowPatch, Arc
import matplotlib.font_manager as fm
import glob, os

OUT = os.path.dirname(os.path.abspath(__file__))
for p in glob.glob("/usr/share/fonts/truetype/nanum/NanumGothic.ttf") + \
         glob.glob(os.path.expanduser("~/.fonts/NanumGothic.ttf")):
    try: fm.fontManager.addfont(p)
    except Exception: pass
plt.rcParams.update({"font.size": 12, "axes.linewidth": 1.0,
                     "font.family": "NanumGothic", "axes.unicode_minus": False})

BLUE, RED, GRAY, GREEN = "#1f6fb2", "#c0392b", "#888888", "#27a35b"


def arc_pts(cx, cy, r, a0, a1, n=200):
    a = np.linspace(np.radians(a0), np.radians(a1), n)
    return cx + r*np.cos(a), cy + r*np.sin(a)


# ============================================================ Fig 1: arc model
def fig_model():
    fig, ax = plt.subplots(figsize=(7.2, 6.4))
    O = (0.0, 0.0)
    R, base = 3.0, 1.4          # inner radius, wheel base (radial gap)
    a0, a1 = 18.0, 78.0          # swept angle (theta)
    Rin, Rout = R, R + base

    # filled slice
    ax.add_patch(Wedge(O, Rout, a0, a1, width=base, facecolor="#eaf2fb",
                       edgecolor="none", zorder=0))

    # two arcs (wheel paths)
    ax.plot(*arc_pts(*O, Rin,  a0, a1), color=RED,  lw=3, zorder=3)   # inner s_L
    ax.plot(*arc_pts(*O, Rout, a0, a1), color=BLUE, lw=3, zorder=3)   # outer s_R

    # two radii (robot axle at start and end)
    for a, lab in ((a0, "start"), (a1, "end")):
        xi, yi = Rin*np.cos(np.radians(a)),  Rin*np.sin(np.radians(a))
        xo, yo = Rout*np.cos(np.radians(a)), Rout*np.sin(np.radians(a))
        ax.plot([0, xo], [0, yo], color=GRAY, lw=1.2, ls="--", zorder=1)
        # axle (the robot) = segment between the two wheels
        ax.plot([xi, xo], [yi, yo], color="k", lw=4, solid_capstyle="round", zorder=4)
        ax.plot(xi, yi, "o", color=RED,  ms=9, zorder=5)
        ax.plot(xo, yo, "o", color=BLUE, ms=9, zorder=5)

    # theta arc at O
    ax.add_patch(Arc(O, 2.0, 2.0, angle=0, theta1=a0, theta2=a1,
                     color="k", lw=1.5))
    am = np.radians((a0+a1)/2)
    ax.annotate(r"$\theta$", (1.15*np.cos(am), 1.15*np.sin(am)),
                fontsize=16, ha="center", va="center")
    ax.plot(*O, "ko", ms=6)
    ax.annotate("O\n(회전 중심)", O, textcoords="offset points",
                xytext=(-6, -18), ha="center", color="k")

    # radius label R (on start radius, inner part)
    am0 = np.radians(a0)
    ax.annotate(r"$R$", (0.5*Rin*np.cos(am0)-0.15, 0.5*Rin*np.sin(am0)),
                color=GRAY, ha="right")

    # arc length labels
    xm_i, ym_i = arc_pts(*O, Rin,  (a0+a1)/2, (a0+a1)/2, 1)
    xm_o, ym_o = arc_pts(*O, Rout, (a0+a1)/2, (a0+a1)/2, 1)
    ax.annotate(r"$s_L$  (안쪽 바퀴)", (xm_i[0], ym_i[0]),
                textcoords="offset points", xytext=(8, -2), color=RED, fontsize=12)
    ax.annotate(r"$s_R$  (바깥쪽 바퀴)", (xm_o[0], ym_o[0]),
                textcoords="offset points", xytext=(8, 4), color=BLUE, fontsize=12)

    # base_L bracket on the 'end' axle
    ax.annotate(r"$base_L$", ((Rin+Rout)/2*np.cos(np.radians(a1)) - 0.1,
                              (Rin+Rout)/2*np.sin(np.radians(a1)) + 0.15),
                color="k", ha="right", fontsize=12)

    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("차동구동 회전: 두 호가 같은 중심각 " r"$\theta$" "를 공유",
                 fontsize=13)
    ax.set_xlim(-1.2, Rout+0.6); ax.set_ylim(-0.8, Rout+0.6)
    fig.tight_layout(); fig.savefig(f"{OUT}/fig1_arc_model.png", dpi=150)
    plt.close(fig)


# ====================================================== Fig 2: theta derivation
def fig_derivation():
    fig, ax = plt.subplots(figsize=(7.6, 5.6))
    O = (0.0, 0.0)
    R, base = 3.2, 1.5
    a0, a1 = 20.0, 70.0
    Rin, Rout = R, R + base

    ax.add_patch(Wedge(O, Rout, a0, a1, width=base, facecolor="#f3f3f3",
                       edgecolor="none", zorder=0))
    ax.plot(*arc_pts(*O, Rin,  a0, a1), color=RED,  lw=3)
    ax.plot(*arc_pts(*O, Rout, a0, a1), color=BLUE, lw=3)
    for a in (a0, a1):
        xo, yo = Rout*np.cos(np.radians(a)), Rout*np.sin(np.radians(a))
        ax.plot([0, xo], [0, yo], color=GRAY, lw=1.2, ls="--")
    ax.add_patch(Arc(O, 1.6, 1.6, theta1=a0, theta2=a1, color="k", lw=1.5))
    am = np.radians((a0+a1)/2)
    ax.annotate(r"$\theta$", (0.95*np.cos(am), 0.95*np.sin(am)), fontsize=15,
                ha="center", va="center")
    ax.plot(*O, "ko", ms=6); ax.annotate("O", O, xytext=(-14, -6),
                                          textcoords="offset points")

    # equations box
    txt = ("\n".join([
        r"호 길이 $= $ 반지름 $\times$ 중심각",
        r"",
        r"$R\,\theta = s_L$",
        r"$(R + base_L)\,\theta = s_R$",
        r"",
        r"두 식을 빼면 $\Rightarrow$  $base_L\,\theta = s_R - s_L$",
    ]))
    ax.text(Rout+0.4, Rout*0.55, txt, fontsize=13, va="center", ha="left",
            bbox=dict(boxstyle="round", fc="#fbfbe8", ec="#bdbd6a"))
    ax.text(Rout+0.4, 0.7,
            r"$\theta = \frac{s_R - s_L}{base_L}$",
            fontsize=20, va="center", ha="left", color="#7a1f1f",
            bbox=dict(boxstyle="round", fc="white", ec="#7a1f1f", lw=2))

    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("두 호 길이 식을 연립하여 " r"$\theta$" " 유도", fontsize=13)
    ax.set_xlim(-1.0, Rout+5.2); ax.set_ylim(-0.6, Rout+0.5)
    fig.tight_layout(); fig.savefig(f"{OUT}/fig2_theta_derivation.png", dpi=150)
    plt.close(fig)


# ======================================================= Fig 3: special cases
def fig_special():
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.6))

    def robot(ax, cx, cy, ang, c="k", L=0.9, W=0.55):
        # draw axle (wheels) perpendicular to heading 'ang'
        dx, dy = np.cos(np.radians(ang)), np.sin(np.radians(ang))
        px, py = -dy, dx
        ax.plot([cx-px*W, cx+px*W], [cy-py*W, cy+py*W], color=c, lw=5,
                solid_capstyle="round")
        ax.add_patch(FancyArrowPatch((cx, cy), (cx+dx*L, cy+dy*L),
                     arrowstyle="-|>", mutation_scale=14, color=c, lw=1.5))

    # (1) straight : s_R = s_L -> theta = 0
    ax = axes[0]
    for x in (-0.6, 0.6):
        ax.add_patch(FancyArrowPatch((x, 0), (x, 2.2), arrowstyle="-|>",
                     mutation_scale=14, color=GRAY, lw=2))
    robot(ax, 0, 0, 90, c="k")
    robot(ax, 0, 2.2, 90, c=GREEN)
    ax.text(0, -0.5, r"$s_R = s_L \;\Rightarrow\; \theta = 0$", ha="center",
            fontsize=13)
    ax.set_title("(1) 직진", fontsize=12)
    ax.set_xlim(-1.6, 1.6); ax.set_ylim(-0.9, 2.7)

    # (2) spin in place : L=+V, R=-V, center at axle midpoint
    ax = axes[1]
    O = (0, 0)
    ax.add_patch(Arc(O, 2.0, 2.0, theta1=-90, theta2=120, color=GRAY, lw=1.2,
                     ls="--"))
    robot(ax, 0, 0, 90, c="k")
    robot(ax, 0, 0, 150, c=GREEN)
    ax.plot(0, 0, "ko", ms=6)
    # opposite wheel arrows
    ax.add_patch(FancyArrowPatch((-0.55, -0.18), (-0.55, 0.5),
                 arrowstyle="-|>", mutation_scale=12, color=RED, lw=2))
    ax.add_patch(FancyArrowPatch((0.55, 0.18), (0.55, -0.5),
                 arrowstyle="-|>", mutation_scale=12, color=BLUE, lw=2))
    ax.text(0, -1.35, r"$L=+V,\; R=-V$" + "\n" + "회전중심 = 차축 중앙",
            ha="center", fontsize=12)
    ax.set_title("(2) 제자리 회전", fontsize=12)
    ax.set_xlim(-1.7, 1.7); ax.set_ylim(-1.9, 1.7)

    # (3) pivot on one wheel : R = 0
    ax = axes[2]
    O = (-0.7, 0)                      # inner (left) wheel = pivot
    ax.plot(*O, "o", color=RED, ms=11)
    ax.add_patch(Arc(O, 2*1.4, 2*1.4, theta1=0, theta2=70, color=GRAY, lw=1.2,
                     ls="--"))
    robot(ax, 0, 0, 90, c="k")
    ex, ey = O[0] + 1.4*np.cos(np.radians(70)), O[1] + 1.4*np.sin(np.radians(70))
    robot(ax, ex, ey, 90+70, c=GREEN)
    ax.text(-0.7, -0.55, "회전축\n(안쪽 바퀴)", ha="center", color=RED,
            fontsize=10)
    ax.text(0.1, -1.4, r"$R = 0$", ha="center", fontsize=13)
    ax.set_title("(3) 한쪽 바퀴 축 회전", fontsize=12)
    ax.set_xlim(-1.8, 2.1); ax.set_ylim(-1.9, 2.1)

    for ax in axes:
        ax.set_aspect("equal"); ax.axis("off")
    fig.suptitle("차동구동 회전의 특수한 경우", fontsize=13)
    fig.tight_layout(); fig.savefig(f"{OUT}/fig3_special_cases.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    fig_model(); fig_derivation(); fig_special()
    print("saved fig1_arc_model.png, fig2_theta_derivation.png, fig3_special_cases.png")
