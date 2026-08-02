# Roadmap

## Immediate next steps (before any performance claims)
- [ ] Benchmark against a baseline (e.g. direct energy-minimization search
  without the Sinkhorn-step structure) across multiple random targets,
  following the statistical methodology established in
  `continuumflow/paper/technical_report_2_evaluation.md` (paired tests,
  reported n, honest limitations).
- [ ] Joint (not per-waypoint) actuation trajectory smoothing.
- [ ] Replace forward-Euler centerline integration with a proper SE(3)
  matrix-exponential integrator (see `docs/known_issues.md`).

## Medium-term
- [ ] Shear and torsion strain DOF (currently only bending + axial).
- [ ] Multi-segment `BodyTree`-style composition (currently single-rod
  only).
- [ ] C++ acceleration for the equilibrium solve's inner energy
  evaluation (the main cost bottleneck, called many times per Sinkhorn
  step).
- [ ] Data-level Jelly bridge (see `docs/jelly_relationship.md`) --
  contingent on confirming Jelly's license permits this kind of
  interop.

## Explicitly out of scope for now
- Reimplementing Jelly's full recursive spatial-vector inverse dynamics
  algorithm — CoSoRoMotion currently uses a simpler energy-minimization
  quasi-static equilibrium, not full forward/inverse dynamics with
  inertial effects. This is a real capability gap relative to Jelly,
  not a hidden equivalence.
