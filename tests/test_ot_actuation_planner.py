import numpy as np
from cosoromotion.gvs.rod import GVSRod
from cosoromotion.planning.ot_actuation_planner import (
    make_equilibrium_tip_cost, plan_actuation_trajectory, OTActuationPlanConfig,
)


def _rod():
    return GVSRod(length=0.2, radius=0.01, youngs_modulus=1e6, n_gauss=6)


def test_equilibrium_cost_is_zero_at_matching_tip():
    rod = _rod()
    straight_tip = rod.tip_position(np.zeros(rod.n_q))
    cost_fn = make_equilibrium_tip_cost(rod, target_tip=straight_tip)
    assert cost_fn(np.zeros(rod.n_q)) < 1e-3


def test_plan_actuation_trajectory_reduces_tip_error():
    rod = _rod()
    straight_tip = rod.tip_position(np.zeros(rod.n_q))
    target = straight_tip + np.array([0.02, 0.0, -0.01])

    cfg = OTActuationPlanConfig.for_rod(rod, n_iters=15)  # keep the test fast
    actuations, histories = plan_actuation_trajectory(
        rod, target_tips=np.array([target]), config=cfg,
    )
    assert actuations.shape == (1, rod.n_q)

    def external_work(q):
        return -float(actuations[0] @ q)
    q_eq, _ = rod.equilibrium(external_work, q_init=np.zeros(rod.n_q))
    achieved_tip = rod.tip_position(q_eq)
    err_after = np.linalg.norm(achieved_tip - target)
    err_before = np.linalg.norm(straight_tip - target)
    assert err_after < err_before
