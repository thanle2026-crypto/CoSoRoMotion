# Known Issues (and fixed issues, kept visible)

Following the practice established in ContinuumFlow's development, we
keep a record of real bugs found during development rather than quietly
fixing and forgetting them -- future contributors extending this code
should know what already went wrong once.

## [FIXED, v0.1.0] Actuation search radius hardcoded at the wrong scale

**Symptom:** `examples/01_ot_actuation_planning.py`, in its first working
version, reported a mean tip tracking error of 134 mm after "optimizing"
actuation for targets only 21.7 mm from the rod's rest position -- i.e.
the optimizer made tracking *worse* than doing nothing.

**Root cause:** `OTActuationPlanConfig`'s default Sinkhorn-step search
radius (`step_radius0`) was initially set to `0.5`, copied directly from
ContinuumFlow's spatial trajectory-optimization examples, where
waypoints are 3D positions in meters (a sensible scale: 0.5 m). Here,
the optimized quantity is a *generalized actuation force* acting on a
soft rod with bending stiffness `EI ≈ 0.0016 N·m²` (deliberately compliant,
representative of a soft actuator). For this rod, a generalized force of
magnitude `0.001` already induces a curvature of `~3 rad/m` (a large,
physically extreme bend for a 0.2 m rod). A search radius of `0.5` is
roughly **500x too large**, causing the optimizer to sample forces large
enough to wrap the rod almost fully around itself, producing near-random
tip positions.

**Fix:** `GVSRod.characteristic_force_scale()` derives a physically
sensible force scale directly from the rod's own stiffness (via the
equilibrium condition `EI * L * kappa = f` for a target curvature), and
`OTActuationPlanConfig.for_rod(rod)` uses it to set the search radius.
The bare `OTActuationPlanConfig()` constructor with no rod-derived scale
is deliberately left non-functional (`sinkhorn_config` stays `None` and
downstream code raises a clear error) rather than falling back to a
silently-wrong hardcoded default -- we'd rather force an explicit,
correctly-scaled config than repeat this bug for a different rod.

**Result after fix:** the same example's tracking error dropped to
6.2 mm (better than the 21.7 mm starting offset, as expected).

**Lesson for future modules:** any time a gradient-free, sampling-based
optimizer's search radius/step size is a hardcoded numeric default, ask
"a scale in what units, appropriate for what physical quantity?" before
reusing it in a new context. This is the second time in this project
family a hardcoded default silently broke behavior in a new context
(the first being ContinuumFlow's shape-selection heuristic bug, see
`continuumflow/paper/technical_report_1.md`) -- a pattern worth naming
explicitly rather than treating each instance as a one-off.

## Open (unfixed) issues

- **Small-angle/forward-Euler centerline integration** in
  `GVSRod.integrate_centerline` is only first-order accurate and departs
  from the analytic constant-curvature arc at high curvature or coarse
  `n_points` (see `test_constant_curvature_matches_pcc_analytic_arc`,
  which explicitly checks convergence rather than exactness at low
  resolution). A proper SE(3) matrix-exponential integrator is the
  correct fix and is planned (see `docs/roadmap.md`).
- **Only bending + axial strain (3 DOF) are modeled**; shear and torsion
  are not yet represented. This limits the model to planar-bending-plus-
  stretch deformations.
- **Per-waypoint (not jointly-smoothed) actuation planning**:
  `plan_actuation_trajectory` optimizes each waypoint's actuation
  independently (warm-started from the previous solution), with no
  explicit smoothness cost across the actuation trajectory the way
  ContinuumFlow's FTL planner has. This is a likely source of jerky
  actuation sequences for longer paths and has not yet been benchmarked.
- **No statistical validation yet.** Unlike ContinuumFlow's technical
  report 2, this project has not yet been benchmarked against a baseline
  or evaluated across multiple random targets/seeds. This is the
  immediate next step before any claims of this approach's advantages
  should be taken seriously.
