#!/usr/bin/env python3
"""
fossen_dynamics.py
==================
3-DOF Fossen Surface Vessel Dynamics Model

Implements the nonlinear equations of motion for a marine surface vessel
in the horizontal plane (surge, sway, yaw) per:
    Fossen, T.I. (2021). Handbook of Marine Craft Hydrodynamics and
    Motion Control, 2nd Ed. John Wiley & Sons.

State:
    η = [x, y, ψ]ᵀ   — Earth-fixed position and heading
    ν = [u, v, r]ᵀ   — Body-fixed surge, sway, yaw-rate

Equations of motion:
    η̇ = R(ψ) ν
    M ν̇ + C(ν) ν + D(ν) ν = τ

Control: τ = [X, Y, N]ᵀ  (surge force, sway force, yaw moment)

Twin-thruster differential drive:
    X = T_port + T_stbd
    N = b * (T_stbd - T_port)   [b = half thruster separation]
"""

import math
import numpy as np


class FossenASVModel:
    """
    3-DOF nonlinear Fossen dynamics model for a twin-thruster ASV.

    Default parameters represent a small ~50 kg ASV (~1.5 m LOA).
    All units: SI (m, kg, N, s, rad).
    """

    def __init__(
        self,
        # ── Rigid body ──────────────────────────────────────────────────────
        mass: float = 50.0,           # [kg]
        Iz: float = 12.0,             # [kg·m²] yaw moment of inertia
        # ── Added mass (hydrodynamic) ────────────────────────────────────────
        Xudot: float = -5.0,          # [kg]    surge added mass
        Yvdot: float = -20.0,         # [kg]    sway added mass
        Nrdot: float = -3.0,          # [kg·m²] yaw added moment
        # ── Linear damping ───────────────────────────────────────────────────
        Xu: float = -10.0,            # [N·s/m]
        Yv: float = -30.0,            # [N·s/m]
        Nr: float = -8.0,             # [N·m·s/rad]
        # ── Quadratic (nonlinear) damping ────────────────────────────────────
        Xuu: float = -5.0,            # [N·s²/m²]
        Yvv: float = -20.0,           # [N·s²/m²]
        Nrr: float = -5.0,            # [N·m·s²/rad²]
        # ── Thruster geometry ────────────────────────────────────────────────
        thruster_separation: float = 0.6,        # [m] port-to-stbd distance
        max_thrust_per_thruster: float = 100.0,  # [N]
        # ── Initial state ────────────────────────────────────────────────────
        x0: float = 0.0, y0: float = 0.0, psi0: float = 0.0,
    ):
        # ── Effective inertia (rigid body + added mass) ──────────────────────
        self.m11 = mass - Xudot   # surge
        self.m22 = mass - Yvdot   # sway
        self.m33 = Iz - Nrdot     # yaw

        # M⁻¹ (diagonal matrix → element-wise reciprocal)
        self.M_inv = np.diag([1.0 / self.m11, 1.0 / self.m22, 1.0 / self.m33])

        # ── Damping coefficients ─────────────────────────────────────────────
        self.Xu, self.Xuu = Xu, Xuu
        self.Yv, self.Yvv = Yv, Yvv
        self.Nr, self.Nrr = Nr, Nrr

        # ── Thruster geometry ────────────────────────────────────────────────
        self.b = thruster_separation / 2.0   # half-separation [m]
        self.max_T = max_thrust_per_thruster

        # ── State ────────────────────────────────────────────────────────────
        self.eta = np.array([x0, y0, psi0], dtype=float)  # [x, y, ψ]
        self.nu  = np.zeros(3, dtype=float)                # [u, v, r]
        self.tau = np.zeros(3, dtype=float)                # last generalised force

    # ── Internal matrices ────────────────────────────────────────────────────

    def _R(self, psi: float) -> np.ndarray:
        """Rotation matrix R(ψ): body → Earth-fixed."""
        c, s = math.cos(psi), math.sin(psi)
        return np.array([[c, -s, 0.0],
                         [s,  c, 0.0],
                         [0.0, 0.0, 1.0]])

    def _C(self, nu: np.ndarray) -> np.ndarray:
        """Coriolis–centripetal matrix C(ν) — Fossen (2021) eq. 3.65."""
        u, v, r = nu
        return np.array([
            [0.0,           0.0,  -self.m22 * v],
            [0.0,           0.0,   self.m11 * u],
            [self.m22 * v, -self.m11 * u,  0.0],
        ])

    def _D(self, nu: np.ndarray) -> np.ndarray:
        """Nonlinear damping D(ν) = D_linear + D_quadratic(ν)."""
        u, v, r = nu
        return np.diag([
            -(self.Xu + self.Xuu * abs(u)),
            -(self.Yv + self.Yvv * abs(v)),
            -(self.Nr + self.Nrr * abs(r)),
        ])

    # ── Integration ──────────────────────────────────────────────────────────

    def step(self, tau: np.ndarray, dt: float):
        """
        Integrate one step using 4th-order Runge–Kutta.

        Args:
            tau: [X, Y, N] generalised forces  [N, N, N·m]
            dt:  time step                      [s]

        Returns:
            (eta_copy, nu_copy) — updated state
        """
        self.tau = tau.copy()

        def f(nu_k, eta_k):
            C = self._C(nu_k)
            D = self._D(nu_k)
            R = self._R(eta_k[2])
            nu_dot  = self.M_inv @ (tau - C @ nu_k - D @ nu_k)
            eta_dot = R @ nu_k
            return nu_dot, eta_dot

        k1_nu, k1_e = f(self.nu, self.eta)
        k2_nu, k2_e = f(self.nu + 0.5*dt*k1_nu, self.eta + 0.5*dt*k1_e)
        k3_nu, k3_e = f(self.nu + 0.5*dt*k2_nu, self.eta + 0.5*dt*k2_e)
        k4_nu, k4_e = f(self.nu +     dt*k3_nu, self.eta +     dt*k3_e)

        self.nu  += (dt / 6.0) * (k1_nu + 2*k2_nu + 2*k3_nu + k4_nu)
        self.eta += (dt / 6.0) * (k1_e  + 2*k2_e  + 2*k3_e  + k4_e)
        self.eta[2] = math.atan2(math.sin(self.eta[2]), math.cos(self.eta[2]))

        return self.eta.copy(), self.nu.copy()

    # ── Thruster allocation ──────────────────────────────────────────────────

    def cmd_vel_to_tau(
        self,
        linear_x: float, angular_z: float,
        surge_gain: float = 50.0, yaw_gain: float = 20.0,
    ) -> np.ndarray:
        """Map cmd_vel inputs to generalised forces τ = [X, 0, N]."""
        return np.array([linear_x * surge_gain, 0.0, angular_z * yaw_gain])

    def tau_to_thrusters(self, tau: np.ndarray):
        """
        Allocate τ to port/starboard thrusters:
            T_port = (X − N/b) / 2
            T_stbd = (X + N/b) / 2

        Returns:
            (T_port, T_stbd) clipped to ±max_thrust
        """
        X, _, N = tau
        T_port = float(np.clip((X - N / self.b) / 2.0, -self.max_T, self.max_T))
        T_stbd = float(np.clip((X + N / self.b) / 2.0, -self.max_T, self.max_T))
        return T_port, T_stbd

    def reset_state(self, x: float, y: float, psi: float,
                    u: float = 0.0, v: float = 0.0, r: float = 0.0):
        """Sync model state with external ground truth."""
        self.eta = np.array([x, y, psi], dtype=float)
        self.nu  = np.array([u, v, r],   dtype=float)
