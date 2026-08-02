import numpy as np
from cosoromotion.gvs.rod import GVSRod, constant_basis, affine_basis


def _make_rod(basis_fn=None):
    return GVSRod(length=0.2, radius=0.01, youngs_modulus=1e6,
                  basis_fn=basis_fn, n_gauss=8)


def test_zero_strain_gives_straight_rod():
    rod = _make_rod()
    q0 = np.zeros(rod.n_q)
    tip = rod.tip_position(q0)
    assert np.allclose(tip, [0, 0, rod.length], atol=1e-6)


def test_constant_curvature_matches_pcc_analytic_arc():
    # With a single constant-basis DOF for kx (kappa_x), the rod should
    # trace a circular arc, matching the closed-form PCC tip position:
    # for pure kx bending, tip = (0, -(1-cos(theta))/kx, sin(theta)/kx)
    # under this module's small-angle/forward-Euler integration scheme
    # (exact only in the limit of many integration points; we check
    # convergence rather than exactness at low n_points).
    rod = _make_rod()
    kx = 2.0  # 1/m
    q = np.array([kx, 0.0, 0.0])  # [kx, ky, eps] constant coefficients
    theta = kx * rod.length
    expected = np.array([0.0, -(1 - np.cos(theta)) / kx, np.sin(theta) / kx])

    tip_coarse = rod.tip_position(q, n_points=10)
    tip_fine = rod.tip_position(q, n_points=400)

    err_coarse = np.linalg.norm(tip_coarse - expected)
    err_fine = np.linalg.norm(tip_fine - expected)
    assert err_fine < err_coarse  # finer integration converges toward the analytic arc
    assert err_fine < 1e-3


def test_affine_basis_has_more_dof_than_constant_basis():
    rod_const = _make_rod(constant_basis(3))
    rod_affine = _make_rod(affine_basis(3))
    assert rod_affine.n_q == 2 * rod_const.n_q


def test_elastic_energy_zero_at_zero_strain():
    rod = _make_rod()
    assert rod.elastic_energy(np.zeros(rod.n_q)) == 0.0


def test_elastic_energy_increases_with_curvature():
    rod = _make_rod()
    e_small = rod.elastic_energy(np.array([0.5, 0.0, 0.0]))
    e_large = rod.elastic_energy(np.array([2.0, 0.0, 0.0]))
    assert e_large > e_small > 0.0


def test_equilibrium_with_no_external_force_returns_to_straight():
    rod = _make_rod()
    q_eq, success = rod.equilibrium(external_work_fn=lambda q: 0.0,
                                     q_init=np.array([1.0, 0.5, 0.0]))
    assert success
    assert np.allclose(q_eq, 0.0, atol=1e-3)


def test_equilibrium_bends_toward_a_lateral_tip_force():
    rod = _make_rod()
    # A force pulling the tip in +x should induce positive kx-like bending
    # (equilibrium minimizes elastic energy - F.tip_position).
    F = np.array([50.0, 0.0, 0.0])  # N, lateral

    def external_work(q):
        tip = rod.tip_position(q, n_points=15)
        return -float(F @ tip)

    q_eq, success = rod.equilibrium(external_work, q_init=np.zeros(rod.n_q))
    assert success
    tip_eq = rod.tip_position(q_eq)
    assert tip_eq[0] > 0.0  # bent toward the applied force direction
