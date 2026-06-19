#!/usr/bin/env python3
"""차동구동 회전 기하 설명용 그림 생성 (보고서용).
labels: math/English (Korean fonts unavailable). Outputs PNGs in this dir.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, FancyArrowPatch, Arc, Polygon
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


def _rectpoly(c, along, across, L, W):
    c = np.asarray(c, float)
    return np.array([c + s1*L/2*along + s2*W/2*across
                     for s1, s2 in [(1, 1), (1, -1), (-1, -1), (-1, 1)]])


def _units(a_deg):
    ar = np.radians(a_deg)
    rad = np.array([np.cos(ar), np.sin(ar)])      # 반경 바깥방향(= 차축 방향)
    head = np.array([-np.sin(ar), np.cos(ar)])    # 접선/진행방향(CCW)
    return rad, head


def mouse_on_arc(ax, R, base, a, alpha=1.0, body_c="#d9e6f2", edge="#33536e",
                 label=False):
    """반경 R(안쪽)·R+base(바깥쪽) 호 위, 중심각 a 위치에 마이크로마우스 배치.
    안쪽 바퀴는 s_L 호, 바깥쪽 바퀴는 s_R 호에 정확히 놓인다."""
    Rout = R + base
    rad, head = _units(a)
    inner, outer, mid = R*rad, Rout*rad, (R+Rout)/2*rad
    ax.add_patch(Polygon(_rectpoly(mid, head, rad, 0.95, base*0.9), closed=True,
                 facecolor=body_c, edgecolor=edge, lw=1.4, alpha=alpha, zorder=5,
                 joinstyle="round"))
    for c, col in ((inner, RED), (outer, BLUE)):
        ax.add_patch(Polygon(_rectpoly(c, head, rad, 0.6, 0.2), closed=True,
                     facecolor=col, edgecolor="k", lw=1.0, alpha=alpha, zorder=7))
    ax.add_patch(FancyArrowPatch(mid, mid + 0.9*head, arrowstyle="-|>",
                 mutation_scale=15, color="#222", lw=1.8, alpha=alpha, zorder=8))
    if label:
        ax.annotate("안쪽 바퀴\n($s_L$ 호)", inner, textcoords="offset points",
                    xytext=(-58, -30), color=RED, fontsize=11, ha="center",
                    arrowprops=dict(arrowstyle="->", color=RED, lw=1.2))
        ax.annotate("바깥쪽 바퀴\n($s_R$ 호)", outer, textcoords="offset points",
                    xytext=(58, 16), color=BLUE, fontsize=11, ha="center",
                    arrowprops=dict(arrowstyle="->", color=BLUE, lw=1.2))


# ============================================================ Fig 1: arc model
def fig_model():
    fig, ax = plt.subplots(figsize=(7.6, 7.0))
    O = (0.0, 0.0)
    R, base = 3.0, 1.3
    a0, a1 = 16.0, 74.0
    Rout = R + base

    ax.add_patch(Wedge(O, Rout, a0, a1, width=base, facecolor="#eef4fb",
                       edgecolor="none", zorder=0))
    ax.plot(*arc_pts(*O, R,    a0, a1), color=RED,  lw=3, zorder=2)   # s_L
    ax.plot(*arc_pts(*O, Rout, a0, a1), color=BLUE, lw=3, zorder=2)   # s_R

    for a in (a0, a1):
        rad, _ = _units(a)
        ax.plot([0, Rout*rad[0]], [0, Rout*rad[1]], color=GRAY, lw=1.1, ls="--",
                zorder=1)
    mouse_on_arc(ax, R, base, a0, alpha=0.45)
    mouse_on_arc(ax, R, base, a1, alpha=1.0, label=True)

    ax.add_patch(Arc(O, 2.1, 2.1, theta1=a0, theta2=a1, color="k", lw=1.5))
    am = np.radians((a0+a1)/2)
    ax.annotate(r"$\theta$", (1.25*np.cos(am), 1.25*np.sin(am)), fontsize=17,
                ha="center", va="center")
    ax.plot(*O, "ko", ms=6)
    ax.annotate("O  (회전 중심)", O, textcoords="offset points", xytext=(8, -16),
                ha="left")
    rad0, _ = _units(a0)
    ax.annotate(r"$R$", (0.55*R*rad0[0]+0.12, 0.55*R*rad0[1]-0.05), color=GRAY)
    p_in, p_out = R*rad0, Rout*rad0
    ax.annotate("", p_out, p_in, arrowprops=dict(arrowstyle="<->", color="k",
                lw=1.4))
    mid0 = (p_in+p_out)/2
    ax.annotate(r"$base_L$", (mid0[0]-0.12, mid0[1]), color="k", ha="right",
                fontsize=12)
    xi, yi = arc_pts(*O, R,    (a0+a1)/2+6, (a0+a1)/2+6, 1)
    xo, yo = arc_pts(*O, Rout, (a0+a1)/2-6, (a0+a1)/2-6, 1)
    ax.annotate(r"$s_L$ (안쪽 호)", (xi[0], yi[0]), color=RED, fontsize=12,
                ha="center")
    ax.annotate(r"$s_R$ (바깥쪽 호)", (xo[0], yo[0]), color=BLUE, fontsize=12,
                ha="center")

    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("차동구동 회전 — 두 바퀴가 같은 중심각 " r"$\theta$" "를\n"
                 "공유하는 두 호를 그린다", fontsize=13)
    ax.set_xlim(-1.3, Rout+0.7); ax.set_ylim(-0.9, Rout+0.8)
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
    mouse_on_arc(ax, R, base, a0, alpha=0.45)     # 시작 자세
    mouse_on_arc(ax, R, base, a1, alpha=1.0)      # 최종 자세
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
def mouse_sc(ax, center, heading_deg, base=1.1, alpha=1.0, body_c="#d9e6f2",
             edge="#33536e", wL=RED, wR=BLUE, arrow=True):
    """마이크로마우스(섀시+양 바퀴+진행화살표). center=차축 중점, heading=진행방향."""
    h = np.array([np.cos(np.radians(heading_deg)), np.sin(np.radians(heading_deg))])
    left = np.array([-h[1], h[0]])         # heading 기준 좌측
    c = np.asarray(center, float)
    ax.add_patch(Polygon(_rectpoly(c, h, left, 0.85, base*0.95), closed=True,
                 facecolor=body_c, edgecolor=edge, lw=1.3, alpha=alpha, zorder=5,
                 joinstyle="round"))
    wl = c + base/2*left
    wr = c - base/2*left
    for wc, col in ((wl, wL), (wr, wR)):
        ax.add_patch(Polygon(_rectpoly(wc, h, left, 0.5, 0.16), closed=True,
                     facecolor=col, edgecolor="k", lw=0.9, alpha=alpha, zorder=7))
    if arrow:
        ax.add_patch(FancyArrowPatch(c, c + 0.8*h, arrowstyle="-|>",
                     mutation_scale=13, color="#222", lw=1.6, alpha=alpha, zorder=8))
    return wl, wr


def fig_special():
    fig, axes = plt.subplots(1, 3, figsize=(14.0, 5.0))
    BASE = 1.1

    # ---------------- (1) 직진 : s_R = s_L -> theta = 0 ----------------
    ax = axes[0]
    for x in (-BASE/2, BASE/2):                       # 양 바퀴 직선 궤적(동일 길이)
        ax.plot([x, x], [0, 2.2], color=GRAY, lw=1.3, ls="--")
        ax.add_patch(FancyArrowPatch((x, 2.0), (x, 2.25), arrowstyle="-|>",
                     mutation_scale=12, color=GRAY, lw=1.3))
    mouse_sc(ax, (0, 0), 90, base=BASE, alpha=0.45)
    mouse_sc(ax, (0, 2.2), 90, base=BASE, body_c="#dff0e4", edge=GREEN)
    ax.text(0, -0.7, r"$s_R = s_L \;\Rightarrow\; \theta = 0$", ha="center",
            fontsize=13)
    ax.set_title("(1) 직진", fontsize=12)
    ax.set_xlim(-1.7, 1.7); ax.set_ylim(-1.1, 3.0)

    # ---------------- (2) 제자리 회전 : 시계방향 120도 ----------------
    ax = axes[1]
    O = np.array([0.0, 0.0])
    rr = 1.45
    # 궤적 점선: 12시(0도)에서 시계방향 120도(4시)까지.  clock->xy: x=rr sin, y=rr cos
    # (180도를 넘기면 좌/우 바퀴가 뒤바뀌어 보이므로 120도로 둔다)
    phi = np.radians(np.linspace(0, 120, 120))
    ax.plot(rr*np.sin(phi), rr*np.cos(phi), color=GRAY, lw=1.3, ls="--")
    ax.add_patch(FancyArrowPatch((rr*np.sin(phi[-2]), rr*np.cos(phi[-2])),
                 (rr*np.sin(phi[-1]), rr*np.cos(phi[-1])), arrowstyle="-|>",
                 mutation_scale=14, color=GRAY, lw=1.4))      # 시계방향 표시
    # 시작 / 최종 둘 다 alpha=0.5 (겹쳐도 둘 다 보이게)
    mouse_sc(ax, O, 90, base=BASE, alpha=0.5)
    mouse_sc(ax, O, 90 - 120, base=BASE, alpha=0.5, body_c="#dff0e4", edge=GREEN)
    ax.plot(*O, "ko", ms=5, zorder=9)
    # 바퀴 속도 방향(L=+V 전진, R=-V 후진) -> 시계방향
    ax.add_patch(FancyArrowPatch((-BASE/2, -0.18), (-BASE/2, 0.55),
                 arrowstyle="-|>", mutation_scale=12, color=RED, lw=2, zorder=9))
    ax.add_patch(FancyArrowPatch((BASE/2, 0.18), (BASE/2, -0.55),
                 arrowstyle="-|>", mutation_scale=12, color=BLUE, lw=2, zorder=9))
    ax.text(0, -1.95, r"$L=+V,\; R=-V$" + "\n" + "회전중심 = 차축 중앙 (시계방향)",
            ha="center", fontsize=11)
    ax.set_title("(2) 제자리 회전", fontsize=12)
    ax.set_xlim(-2.0, 2.0); ax.set_ylim(-2.5, 2.0)

    # ---------------- (3) 한쪽 바퀴 축 회전 : R = 0 ----------------
    ax = axes[2]
    P = np.array([-0.55, -0.35])           # 안쪽(왼쪽) 바퀴 = 회전축 (고정)
    Bp = 1.6                                # 이 패널 바퀴 간격(= 오른쪽바퀴 회전반경)
    alpha_pivot = 78.0                      # 축 기준 회전각

    def mouse_pivot(headeg, **kw):
        # 왼쪽 바퀴가 P에 오도록 center 배치.  left = (-sin,cos)*... ; center = P - base/2*left
        h = np.array([np.cos(np.radians(headeg)), np.sin(np.radians(headeg))])
        left = np.array([-h[1], h[0]])
        center = P + Bp/2*(-left)          # = P + base/2*right  (왼쪽바퀴=center+base/2*left=P)
        return mouse_sc(ax, center, headeg, base=Bp, **kw)

    # 점선 = 오른쪽 바퀴 궤적: 반경 Bp, 중심 P, 0도(오른쪽,3시)에서 alpha까지
    th = np.radians(np.linspace(0, alpha_pivot, 120))
    ax.plot(P[0] + Bp*np.cos(th), P[1] + Bp*np.sin(th), color=GRAY, lw=1.3, ls="--")
    ax.add_patch(FancyArrowPatch(
        (P[0]+Bp*np.cos(th[-2]), P[1]+Bp*np.sin(th[-2])),
        (P[0]+Bp*np.cos(th[-1]), P[1]+Bp*np.sin(th[-1])),
        arrowstyle="-|>", mutation_scale=14, color=GRAY, lw=1.4))
    # 시작(검정, heading 90 = 차축이 +x) / 최종(초록, heading 90+alpha)
    mouse_pivot(90, alpha=0.5)
    mouse_pivot(90 + alpha_pivot, body_c="#dff0e4", edge=GREEN)
    ax.plot(*P, "o", color=RED, ms=12, zorder=10)        # 고정 회전축(겹친 왼쪽바퀴)
    ax.annotate("회전축\n(안쪽 바퀴, 고정)", P, textcoords="offset points",
                xytext=(-6, -34), ha="center", color=RED, fontsize=10)
    ax.text(1.1, -1.7, r"$R = 0$", ha="center", fontsize=13)
    ax.set_title("(3) 한쪽 바퀴 축 회전", fontsize=12)
    ax.set_xlim(-1.8, 2.4); ax.set_ylim(-2.2, 2.2)

    for ax in axes:
        ax.set_aspect("equal"); ax.axis("off")
    fig.suptitle("차동구동 회전의 특수한 경우", fontsize=13)
    fig.tight_layout(); fig.savefig(f"{OUT}/fig3_special_cases.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    fig_model(); fig_derivation(); fig_special()
    print("saved fig1_arc_model.png, fig2_theta_derivation.png, fig3_special_cases.png")
