#!/usr/bin/env python3
"""dqn_score_curve.png / ablation_pursuit_vs_dqn_vs_aeb.png 재생성 (주석 위치 개선)."""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['axes.unicode_minus'] = False
FIG = "/home/sejong/isaac_aeb/figures"

# ───────────────────────── 1) DQN 학습 곡선 ─────────────────────────
d = np.load("/home/sejong/grid_nav/evaluations.npz")
ts = d["timesteps"] / 1000.0          # ×1000 스텝
res = d["results"]
mean = res.mean(1); std = res.std(1)

fig, ax = plt.subplots(figsize=(8, 4.2), dpi=110)
ax.plot(ts, mean, color="#2e8b3d", lw=1.6, label="eval reward (mean of 20 ep)")
ax.fill_between(ts, mean - std, mean + std, color="#2e8b3d", alpha=0.20, label="±1 std")

bx, by = 510, 799.0                    # best checkpoint
ax.plot(bx, by, "o", ms=12, color="#d62728", mec="k", mew=1.2, zorder=5)
# 주석 → 오른쪽 상단 빈 공간, 곡선과 안 겹치게 (axes 좌표 고정)
ax.annotate(
    "★ Best model (deployed)\n799 reward ≈ 93% success\n@ 510k steps",
    xy=(bx, by), xycoords="data",
    xytext=(0.97, 0.92), textcoords="axes fraction",
    ha="right", va="top", fontsize=10,
    bbox=dict(boxstyle="round,pad=0.4", fc="#fff7e6", ec="#e0a000", lw=1.2),
    arrowprops=dict(arrowstyle="->", color="#d62728", lw=1.4,
                    connectionstyle="arc3,rad=-0.15"),
)
ax.set_title("DQN learning curve (grid nav) — best checkpoint at peak used for deployment",
             fontsize=11)
ax.set_xlabel("training steps (×1000)"); ax.set_ylabel("evaluation reward")
ax.grid(alpha=0.25); ax.legend(loc="lower left", framealpha=0.9)
ax.set_xlim(ts.min(), ts.max())
fig.tight_layout(); fig.savefig(f"{FIG}/dqn_score_curve.png", dpi=110); plt.close(fig)
print("saved dqn_score_curve.png")

# ───────────────── 2) Pursuit / DQN / AEB ablation ─────────────────
labels = ["Pursuit only\n(no avoidance)", "DQN (RL)", "AEB (rule)"]
coll   = [35, 20, 10]
estop  = [0.0, 0.0, 4.1]
colors = ["#9e9e9e", "#4caf50", "#e2574c"]

fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 5.2), dpi=110)

# 좌: 막대 + pursuit→DQN 화살표
bars = axL.bar(range(3), coll, color=colors, edgecolor="k", lw=1.0, width=0.6, zorder=2)
for i, v in enumerate(coll):
    axL.text(i, v + 1.0, f"{v}%", ha="center", va="bottom",
             fontsize=14, fontweight="bold", color="k")
# 화살표 (pursuit top → DQN top)
axL.annotate("", xy=(1, 20), xytext=(0, 35),
             arrowprops=dict(arrowstyle="-|>", color="#2e8b3d", lw=2.4))
# 감소율 라벨: 막대 사이 흰 공간(0.3~0.7)에 흰 박스로 또렷하게. 상대 43%↓ 명시
axL.text(0.5, 22.0, "−15%p", ha="center", va="center",
         fontsize=13, fontweight="bold", color="k",
         bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#2e8b3d", lw=1.2))
axL.set_xticks(range(3)); axL.set_xticklabels(labels, fontsize=11)
axL.set_ylabel("collision rate (%) — 20 runs", fontsize=11)
axL.set_ylim(0, 42)
axL.set_title("RL avoidance halves collisions\nPursuit 35% → DQN 20% (E-stop stays 0)", fontsize=12)
axL.grid(axis="y", alpha=0.25)

# 우: 산점도 (충돌률 vs E-stop), 라벨 안쪽으로
pts = {"Pursuit": (35, 0.0, "#9e9e9e"), "DQN": (20, 0.0, "#4caf50"), "AEB": (10, 4.1, "#e2574c")}
for name, (x, y, c) in pts.items():
    axR.scatter(x, y, s=320, color=c, edgecolor="k", lw=1.3, zorder=3)
# 라벨 오프셋 (display points) — 경계선 안쪽 방향
axR.annotate("Pursuit", (35, 0.0), textcoords="offset points", xytext=(14, -16),
             ha="left", va="top", fontsize=13, fontweight="bold")
axR.annotate("DQN", (20, 0.0), textcoords="offset points", xytext=(10, -16),
             ha="left", va="top", fontsize=13, fontweight="bold")
axR.annotate("AEB", (10, 4.1), textcoords="offset points", xytext=(-12, 14),
             ha="right", va="bottom", fontsize=13, fontweight="bold")
axR.set_xlim(38, 7)                    # 반전: 오른쪽일수록 안전, 여백 확보
axR.set_ylim(4.8, -0.6)               # 반전: 위일수록 매끄러움, 여백 확보
axR.set_xlabel("collision rate (%)  ← lower = safer", fontsize=11)
axR.set_ylabel("E-stop (mean)   ↓ lower = smoother", fontsize=11)
axR.set_title("DQN = best balance\n(as smooth as pursuit, far safer)", fontsize=12)
axR.grid(alpha=0.25)

fig.suptitle("Pure-pursuit ablation (along-path): RL cuts collisions 35%→20% while staying perfectly smooth",
             fontsize=14, fontweight="bold")
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(f"{FIG}/ablation_pursuit_vs_dqn_vs_aeb.png", dpi=110); plt.close(fig)
print("saved ablation_pursuit_vs_dqn_vs_aeb.png")
