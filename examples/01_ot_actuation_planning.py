"""Example: plan a sequence of actuation forces, via the Sinkhorn-step
optimal-transport optimizer, so a physically-modeled GVS soft rod's
quasi-static equilibrium tip position tracks a target path -- the core
CoSoRoMotion capability (OT-based planning through a real mechanics
model, not just kinematics)."""
import numpy as np
import matplotlib.pyplot as plt

from cosoromotion.gvs.rod import GVSRod
from cosoromotion.planning.ot_actuation_planner import (
    plan_actuation_trajectory, OTActuationPlanConfig,
)

rod = GVSRod(length=0.2, radius=0.008, youngs_modulus=5e5, n_gauss=6)
straight_tip = rod.tip_position(np.zeros(rod.n_q))
print(f"Rod at zero actuation, tip = {straight_tip}")

# A small set of target tip positions representing a short reaching motion.
targets = straight_tip + np.array([
    [0.01, 0.00, -0.005],
    [0.02, 0.01, -0.010],
    [0.02, 0.02, -0.008],
])

cfg = OTActuationPlanConfig.for_rod(rod, n_iters=30)

actuations, histories = plan_actuation_trajectory(rod, targets, config=cfg)

achieved_tips = []
for f_ext in actuations:
    def external_work(q, f_ext=f_ext):
        return -float(f_ext @ q)
    q_eq, ok = rod.equilibrium(external_work, q_init=np.zeros(rod.n_q))
    achieved_tips.append(rod.tip_position(q_eq))
achieved_tips = np.array(achieved_tips)

errors = np.linalg.norm(achieved_tips - targets, axis=1)
print(f"Mean tip tracking error after OT-based actuation planning: "
      f"{errors.mean()*1000:.2f} mm (targets offset {np.linalg.norm(targets-straight_tip,axis=1).mean()*1000:.1f} mm from rest)")

fig = plt.figure(figsize=(10, 4.5))
ax1 = fig.add_subplot(121, projection="3d")
ax1.plot(*straight_tip[None].repeat(1, axis=0).T, "ko", label="rest tip")
ax1.plot(*targets.T, "r*--", markersize=12, label="targets")
ax1.plot(*achieved_tips.T, "go-", label="achieved (via OT-planned actuation)")
ax1.legend(fontsize=8)
ax1.set_title("Equilibrium tip tracking via OT-planned actuation")

ax2 = fig.add_subplot(122)
for i, h in enumerate(histories):
    ax2.plot(h, label=f"waypoint {i}")
ax2.set_xlabel("Sinkhorn-step iteration")
ax2.set_ylabel("waypoint cost")
ax2.set_title("Per-waypoint cost convergence")
ax2.legend(fontsize=8)

plt.tight_layout()
plt.savefig("example1_ot_actuation_planning.png", dpi=150)
print("Saved example1_ot_actuation_planning.png")
