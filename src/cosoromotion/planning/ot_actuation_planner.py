"""
Optimal-transport-based actuation trajectory planning for GVS soft
robots -- the core "Optimal Transport via Continuum Soft Robot Motion
Planning" capability of CoSoRoMotion.

Unlike ContinuumFlow's shape-library FTL planner (which searches over
*kinematically* achievable shapes directly), this planner searches over
*actuation* trajectories for a GVS rod, where each candidate actuation
must be passed through the rod's quasi-static equilibrium solve (a
nonlinear energy minimization, `gvs.rod.GVSRod.equilibrium`) to find out
what shape it actually produces. This makes the cost function expensive
and only available via evaluation (never a usable analytic gradient,
since it is defined through a nested optimization), which is exactly the
regime ContinuumFlow's Sinkhorn-step optimizer (entropic-OT-based,
gradient-free, evaluation-only) was designed for. This module reuses
that optimizer directly from the `continuumflow` package (a dependency,
not a copy) rather than reimplementing it, keeping the two projects
cleanly layered: `continuumflow.optim` is the general-purpose,
robot-agnostic gradient-free optimizer; `cosoromotion` supplies the
soft-robot-specific, mechanics-aware cost function it optimizes.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable
import numpy as np

from continuumflow.optim.sinkhorn_step import sinkhorn_step_optimize, SinkhornStepConfig

from ..gvs.rod import GVSRod


@dataclass
class OTActuationPlanConfig:
    sinkhorn_config: SinkhornStepConfig = None
    tip_weight: float = 1.0
    obstacle_weight: float = 5.0
    target_curvature_scale: float = 2.0  # used only if sinkhorn_config is None

    def __post_init__(self):
        # NOTE: step_radius0 is intentionally NOT hardcoded here. An
        # earlier version of this config defaulted to step_radius0=0.5,
        # copied from continuumflow's meter-scale spatial trajectory
        # examples. For a compliant GVS rod, the right unit for a
        # *generalized actuation force* search radius is orders of
        # magnitude smaller (see GVSRod.characteristic_force_scale) and
        # depends on the specific rod's stiffness -- a fixed numeric
        # default silently produced actuation samples large enough to
        # wrap the rod almost fully around itself, and the resulting
        # "optimized" trajectory was worse than doing nothing (documented
        # in docs/known_issues.md). If sinkhorn_config is not supplied,
        # callers MUST provide a rod via `for_rod()` so the scale is
        # derived correctly; the bare default below is deliberately left
        # unset (None) to force this.
        pass

    @classmethod
    def for_rod(cls, rod, target_curvature_scale: float = 2.0,
                tip_weight: float = 1.0, obstacle_weight: float = 5.0,
                n_iters: int = 40) -> "OTActuationPlanConfig":
        """Build a config with a search radius derived from the rod's
        own stiffness, rather than a hardcoded, rod-agnostic constant."""
        force_scale = rod.characteristic_force_scale(target_curvature_scale)
        sinkhorn_cfg = SinkhornStepConfig(
            step_radius0=force_scale,
            step_radius_min=force_scale * 0.02,
            decay=0.92,
            epsilon=force_scale * 0.1,
            n_iters=n_iters,
        )
        cfg = cls(sinkhorn_config=sinkhorn_cfg, tip_weight=tip_weight,
                   obstacle_weight=obstacle_weight,
                   target_curvature_scale=target_curvature_scale)
        return cfg


def make_equilibrium_tip_cost(rod: GVSRod, target_tip: np.ndarray,
                               obstacles: list | None = None,
                               config: OTActuationPlanConfig | None = None
                               ) -> Callable[[np.ndarray], float]:
    """Build a per-waypoint cost function over *actuation-consistent
    generalized forces* `f_ext` (an n_q-dim vector representing an
    actuation-induced generalized force, e.g. from tendon tension or
    pressure, mapped through the rod's basis functions) that:
      1. solves the rod's quasi-static equilibrium under that actuation,
      2. scores the resulting tip position against a target and any
         obstacles.

    This is the "black-box, evaluation-only" cost the Sinkhorn-step
    optimizer consumes -- note there is no analytic gradient of this
    cost with respect to `f_ext`, since it is defined through a nested
    nonlinear equilibrium solve.
    """
    cfg = config or OTActuationPlanConfig.for_rod(rod)
    if cfg.sinkhorn_config is None:
        raise ValueError(
            "OTActuationPlanConfig.sinkhorn_config is unset. Build the "
            "config via OTActuationPlanConfig.for_rod(rod) so the search "
            "radius is derived from this rod's stiffness -- see the "
            "docstring on OTActuationPlanConfig for why a hardcoded "
            "default is deliberately not provided."
        )
    obstacles = obstacles or []

    def cost_fn(f_ext: np.ndarray) -> float:
        def external_work(q):
            return -float(f_ext @ q)  # generalized-force work term

        q_eq, success = rod.equilibrium(external_work, q_init=np.zeros(rod.n_q))
        if not success:
            return 1e3  # penalize non-converged equilibria heavily

        tip = rod.tip_position(q_eq)
        c = cfg.tip_weight * float(np.linalg.norm(tip - target_tip))
        for obs in obstacles:
            c += cfg.obstacle_weight * obs.cost(tip)
        return c

    return cost_fn


def plan_actuation_trajectory(rod: GVSRod, target_tips: np.ndarray,
                               obstacles: list | None = None,
                               config: OTActuationPlanConfig | None = None
                               ) -> tuple[np.ndarray, list]:
    """Plan a sequence of generalized actuation forces (one per target
    tip waypoint) using the Sinkhorn-step optimizer, where each waypoint
    is independently optimized against its own equilibrium-tip cost
    (a simple per-waypoint scheme; jointly optimizing the whole
    actuation trajectory with an added smoothness term across waypoints
    is a natural extension, mirroring the smoothness term already used
    in continuumflow's FTL planner).

    Returns (actuation_trajectory, per_waypoint_cost_histories).
    """
    cfg = config or OTActuationPlanConfig.for_rod(rod)
    if cfg.sinkhorn_config is None:
        raise ValueError(
            "OTActuationPlanConfig.sinkhorn_config is unset. Build the "
            "config via OTActuationPlanConfig.for_rod(rod)."
        )
    n_wp = len(target_tips)
    actuations = np.zeros((n_wp, rod.n_q))
    histories = []

    f_init = np.zeros(rod.n_q)
    for i, target in enumerate(target_tips):
        cost_fn = make_equilibrium_tip_cost(rod, target, obstacles, cfg)
        # Treat this single waypoint's actuation vector as a 1-point
        # "trajectory" so we can reuse sinkhorn_step_optimize directly;
        # fixed_endpoints=False so the single point is itself optimized.
        traj_init = f_init[None, :].repeat(3, axis=0)  # need >=3 pts for interior optimization
        opt_traj, history = sinkhorn_step_optimize(
            traj_init, cost_fn, cfg.sinkhorn_config, fixed_endpoints=False,
        )
        best = opt_traj[np.argmin([cost_fn(p) for p in opt_traj])]
        actuations[i] = best
        histories.append(history)
        f_init = best  # warm-start the next waypoint

    return actuations, histories
