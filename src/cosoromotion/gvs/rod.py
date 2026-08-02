"""
Geometric Variable Strain (GVS) rod model.

This is an INDEPENDENT, from-scratch implementation of the strain-
parameterization approach to continuum/soft robot modeling, built from
the published theory it is based on -- not from, and not derived from,
any specific library's source code. Two public references informed the
modeling approach used here:

  - F. Boyer, V. Lebastard, F. Candelier, F. Renda, "Dynamics of
    continuum and soft robots: A strain parameterization based
    approach," IEEE Trans. on Robotics, 37(3):847-863, 2020.
  - P. Pustina, C. Della Santina, A. De Luca, on unified/recursive
    inverse dynamics of modular serial soft-rigid mechanical systems
    (see paper/references.bib for the exact citation).

Where a rod's cross-sectional strain (curvature, shear, elongation) is
represented not as a single constant per segment (as in ContinuumFlow's
PCC model) but as a linear combination of user-chosen basis functions
over arclength, with coefficients `q` as the generalized configuration
variables -- the "geometric variable strain" idea. A constant-basis GVS
rod reduces exactly to PCC; an affine-basis GVS rod captures curvature
that varies linearly along the rod (matching the intuition behind
Jelly's PAC -- piecewise affine curvature -- bodies, though this is an
independent implementation, not a port of Jelly's PAC3D class).

Unlike ContinuumFlow's kinematics-only PCC model, this module computes
*quasi-static mechanical equilibrium*: given a rod's material properties
and a set of external/actuation generalized forces, it solves for the
strain coefficients that minimize total potential energy (elastic strain
energy plus external work), rather than treating strain as a free
kinematic input. This is a genuine capability step beyond pure
kinematics, at the cost of needing an equilibrium solve (nonlinear
minimization) rather than a closed-form evaluation.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable
import numpy as np
from scipy.optimize import minimize

_EPS = 1e-9


def constant_basis(n_dof: int) -> Callable[[float, float], np.ndarray]:
    """A single constant basis function per strain DOF: phi(s) = 1 for
    all s in [0, L]. Reduces the GVS rod to a PCC-equivalent model."""
    def basis(s: float, L: float) -> np.ndarray:
        return np.ones(n_dof)
    return basis


def affine_basis(n_dof: int) -> Callable[[float, float], np.ndarray]:
    """Two basis functions per strain DOF -- constant and linear-in-s/L --
    giving each strain component an affine (not just constant) profile
    along the rod's arclength. Returns a (2*n_dof,)-shaped basis vector;
    q must then have 2*n_dof entries per this rod."""
    def basis(s: float, L: float) -> np.ndarray:
        frac = s / max(L, _EPS)
        const = np.ones(n_dof)
        linear = np.full(n_dof, frac)
        return np.concatenate([const, linear])
    return basis


@dataclass
class GVSRod:
    """A single-segment rod whose strain field is `Phi(s) @ q`, where
    `Phi(s)` stacks the chosen basis functions for each of the 3 strain
    components (kappa_x, kappa_y, elongation epsilon -- a 2D-bending +
    axial model; torsion/shear are a natural extension left for future
    work, matching the roadmap-style scoping used across this platform).
    """
    length: float               # rest length, m
    radius: float                # rod radius, m (uniform circular cross-section)
    youngs_modulus: float        # Pa
    density: float = 1000.0      # kg/m^3 (used only if gravity is included)
    basis_fn: Callable[[float, float], np.ndarray] = field(default=None)
    n_strain_dof: int = 3        # kappa_x, kappa_y, epsilon (axial strain)
    n_gauss: int = 8

    def __post_init__(self):
        if self.basis_fn is None:
            self.basis_fn = constant_basis(self.n_strain_dof)
        # Determine n_q by evaluating the basis once at s=0.
        self.n_q = len(self.basis_fn(0.0, self.length))

        area = np.pi * self.radius ** 2
        I = np.pi * self.radius ** 4 / 4.0  # second moment of area (circular)
        self.EI = self.youngs_modulus * I          # bending stiffness
        self.EA = self.youngs_modulus * area        # axial stiffness

        # Gauss-Legendre quadrature nodes/weights on [0, L].
        nodes, weights = np.polynomial.legendre.leggauss(self.n_gauss)
        self._gauss_s = 0.5 * self.length * (nodes + 1.0)
        self._gauss_w = 0.5 * self.length * weights

    def strain_at(self, s: float, q: np.ndarray) -> np.ndarray:
        """Local strain vector [kappa_x, kappa_y, epsilon] at arclength s,
        given generalized strain coordinates q (length n_q * n_strain_dof,
        stacked as [dof0_coeffs..., dof1_coeffs..., dof2_coeffs...] to
        match `basis_fn`'s (n_strain_dof * n_basis,)-shaped output when
        basis_fn is applied per-DOF; see `_unpack_q`)."""
        Phi = self.basis_fn(s, self.length)  # shape (n_strain_dof * n_basis_per_dof,)
        # q has the same length as Phi by construction (see __post_init__).
        n_basis_per_dof = self.n_q // self.n_strain_dof
        strain = np.zeros(self.n_strain_dof)
        for d in range(self.n_strain_dof):
            phi_d = Phi[d * n_basis_per_dof:(d + 1) * n_basis_per_dof]
            q_d = q[d * n_basis_per_dof:(d + 1) * n_basis_per_dof]
            strain[d] = float(np.dot(phi_d, q_d))
        return strain

    def integrate_centerline(self, q: np.ndarray, n_points: int = 30) -> np.ndarray:
        """Integrate the strain field to obtain the rod's 3D centerline
        under the small-curvature planar-bending-plus-axial-strain
        approximation used throughout this module (kappa_x, kappa_y treat
        bending about two orthogonal axes; full SE(3) strain integration
        via the matrix exponential is a natural higher-fidelity extension,
        left for future work -- see docs/limitations in cosoromotion)."""
        s_vals = np.linspace(0.0, self.length, n_points)
        pts = np.zeros((n_points, 3))
        # Simple forward-Euler integration of position given local strain;
        # adequate for the small-to-moderate curvature regime used in the
        # examples/tests here. A higher-order (e.g. RK4 on the SE(3) ODE)
        # integrator is a documented future improvement.
        pos = np.zeros(3)
        heading = np.zeros(3)  # (theta_x, theta_y) cumulative bend, small-angle
        ds = self.length / max(n_points - 1, 1)
        pts[0] = pos
        for i in range(1, n_points):
            s = s_vals[i]
            kx, ky, eps = self.strain_at(s, q)
            heading[0] += kx * ds
            heading[1] += ky * ds
            dz = (1.0 + eps) * ds
            dx = np.sin(heading[1]) * dz
            dy = -np.sin(heading[0]) * dz
            dz_eff = np.cos(heading[0]) * np.cos(heading[1]) * dz
            pos = pos + np.array([dx, dy, dz_eff])
            pts[i] = pos
        return pts

    def tip_position(self, q: np.ndarray, n_points: int = 30) -> np.ndarray:
        return self.integrate_centerline(q, n_points)[-1]

    def elastic_energy(self, q: np.ndarray) -> float:
        """Total elastic strain energy, via Gaussian quadrature over the
        rod, using linear bending/axial constitutive laws (Hookean rod):
        U = 0.5 * integral( EI*(kx^2 + ky^2) + EA*eps^2 ) ds."""
        U = 0.0
        for s, w in zip(self._gauss_s, self._gauss_w):
            kx, ky, eps = self.strain_at(s, q)
            U += w * (self.EI * (kx ** 2 + ky ** 2) + self.EA * eps ** 2)
        return 0.5 * U

    def characteristic_force_scale(self, target_curvature: float = 2.0) -> float:
        """A physically-derived scale for 'how large a generalized
        actuation force is needed to reach a moderate curvature' for this
        rod, used to set a sensible default search radius for gradient-
        free optimizers acting on actuation forces (see
        planning/ot_actuation_planner.py). Derived from the equilibrium
        condition for constant curvature under a pure generalized-force
        load: at equilibrium, EI * L * kappa = f, so f for a target
        curvature is EI * L * target_curvature. Without this, a fixed
        numeric search radius tuned for one rod's stiffness silently
        breaks for any other rod -- this was a real bug caught during
        development (see docs/known_issues.md)."""
        return self.EI * self.length * target_curvature

    def equilibrium(self, external_work_fn: Callable[[np.ndarray], float],
                     q_init: np.ndarray | None = None,
                     method: str = "L-BFGS-B") -> tuple[np.ndarray, bool]:
        """Solve for the strain coefficients q minimizing total potential
        energy: elastic energy plus external work (e.g. -F . tip_position
        for a tip load, or an actuation-consistency penalty). Returns
        (q_equilibrium, success_flag)."""
        q0 = np.zeros(self.n_q) if q_init is None else np.array(q_init, dtype=float)

        def total_potential(q):
            return self.elastic_energy(q) + external_work_fn(q)

        res = minimize(total_potential, q0, method=method)
        return res.x, bool(res.success)
