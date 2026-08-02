# CoSoRoMotion

**Continuum Soft Robot Motion Planning via Optimal Transport**, built on
an independent Geometric Variable Strain (GVS) dynamics core and
ContinuumFlow's gradient-free Sinkhorn-step optimizer.

> ⚠️ Early-stage (v0.1.0, pre-alpha). See
> [`docs/known_issues.md`](docs/known_issues.md) for what's fixed, what's
> broken, and what's unvalidated. See
> [`docs/jelly_relationship.md`](docs/jelly_relationship.md) for an
> explicit statement of this project's relationship to
> [Jelly](https://piepustina.github.io/Jelly/) — read it before assuming
> any code compatibility.

## What this is

Most of ContinuumFlow (the sibling project this depends on) plans over
robot **shape** directly — given actuator commands, what shape does the
robot have, kinematically. CoSoRoMotion instead plans over **actuation**,
passed through a real (if simplified) **mechanics** model: given a
generalized actuation force and a rod's material properties, what shape
does it settle into at quasi-static equilibrium (minimizing elastic
energy plus external work)? This equilibrium solve is a nonlinear
optimization with no usable analytic gradient with respect to the
actuation — exactly the setting ContinuumFlow's entropic-optimal-
transport-based Sinkhorn-step optimizer was built for, and this project
reuses it directly rather than reimplementing it.

## Install

```bash
git clone <this-repo>
cd cosoromotion
pip install -e ".[dev]"   # requires continuumflow installed alongside (see pyproject.toml)
```

CoSoRoMotion depends on `continuumflow` for its gradient-free optimizer
(`continuumflow.optim.sinkhorn_step`) — install both projects side by
side, or adjust the path dependency in `pyproject.toml`.

## Quickstart

```python
import numpy as np
from cosoromotion.gvs.rod import GVSRod
from cosoromotion.planning.ot_actuation_planner import (
    plan_actuation_trajectory, OTActuationPlanConfig,
)

rod = GVSRod(length=0.2, radius=0.008, youngs_modulus=5e5)
target_tips = np.array([[0.02, 0.01, 0.19]])  # a target tip position

# IMPORTANT: always build the config via for_rod() -- see
# docs/known_issues.md for why a bare, rod-agnostic default is
# deliberately not provided.
cfg = OTActuationPlanConfig.for_rod(rod)
actuations, histories = plan_actuation_trajectory(rod, target_tips, config=cfg)
```

See `examples/01_ot_actuation_planning.py` for a full runnable example
with plotting.

## Architecture

```
cosoromotion/
├── gvs/        # Geometric Variable Strain rod model: strain basis
│               # functions, centerline integration, elastic energy,
│               # quasi-static equilibrium solve
└── planning/   # OT-based actuation trajectory planning, built on
                # continuumflow.optim.sinkhorn_step
```

## Relationship to ContinuumFlow

CoSoRoMotion is a sibling project, not a fork: `continuumflow` remains
the general-purpose, robot-agnostic planning/optimization platform
(shape-library FTL planning, the Sinkhorn-step optimizer itself, learned
residual dynamics); `cosoromotion` supplies a soft-robot-specific,
mechanics-aware dynamics model and the actuation-planning layer built on
top of it. If you only need kinematics (no material properties, no
equilibrium solve), use `continuumflow` directly — it's simpler and
faster.

## License

Apache License 2.0 — see [`LICENSE`](LICENSE). (Note: this covers
CoSoRoMotion's own code only. See `docs/jelly_relationship.md` regarding
Jelly's separate, unconfirmed licensing.)
