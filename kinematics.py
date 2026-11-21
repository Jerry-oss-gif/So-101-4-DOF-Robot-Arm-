"""SO-101 4-DOF kinematics (pan, shoulder, elbow, wrist-flex). Units: m, rad.

Standard DH (angles in rad, lengths in m):
    joint | theta       | d  | a  | alpha
      1   | q1 + 90deg  | d1 | a1 | -90deg   (pan, +90deg home offset)
      2   | q2          | 0  | a2 | 0
      3   | q3          | 0  | a3 | 0
      4   | q4          | 0  | a4 | 0
Conventions: at q = 0 the arm is stretched out along +Y; positive q2/q3/q4 tilt DOWN.
Tool pitch phi = q2 + q3 + q4 (positive = tool pointing down).
"""
from dataclasses import dataclass
import numpy as np

@dataclass
class ArmGeometry:
    d1: float = 0.075   # base height to shoulder axis (m)
    a1: float = 0.075   # horizontal offset base axis -> shoulder axis (m)
    a2: float = 0.130   # upper arm (m)
    a3: float = 0.120   # forearm (m)
    a4: float = 0.065   # wrist -> TCP (m)
    # pan, shoulder, elbow, wrist
    limits_deg: tuple = ((-65, 65), (-180, -55), (23.5, 180), (-80, 80))
    # home = folded pose inside limits
    home_deg: tuple = (0, -143, 99, 67)
    # calibration pose = "upright": upper arm vertical, forearm horizontal forward, wrist level.
    # Hold the real arm in this pose when calibrating (GUI dialog, calibrate_home.py and diagnose.py all use it).
    calib_deg: tuple = (0, -143, 99, 67)

@dataclass
class SingularityStatus:
    index: float      # sigma_min / sigma_max of the task Jacobian (0..1)
    level: str        # "ok" | "warn" | "stop"
    message: str

class SO101Kinematics:
    def __init__(self, geometry=None, pitch_weight=0.1, warn=0.12, stop=0.04, damping=0.02):
        self.g = geometry or ArmGeometry()
        self.limits = np.radians(self.g.limits_deg)
        self.home = np.radians(self.g.home_deg)
        self.calib = np.radians(self.g.calib_deg)
        self.pitch_w, self.warn, self.stop, self.damping = pitch_weight, warn, stop, damping

    @property
    def reach(self):
        return self.g.a1 + self.g.a2 + self.g.a3 + self.g.a4

    # ---------- homogeneous transforms ----------
    @staticmethod
    def dh(a, alpha, d, theta):
        ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,     sa,       ca,      d],
            [0.0,    0.0,      0.0,    1.0],
        ])

    def transforms(self, q):
        """[T00, T01, T02, T03, T04] cumulative base->frame i."""
        g = self.g
        params = [
            (g.a1, -np.pi / 2.0, g.d1, q[0] + np.pi / 2.0),
            (g.a2, 0.0, 0.0, q[1]),
            (g.a3, 0.0, 0.0, q[2]),
            (g.a4, 0.0, 0.0, q[3]),
        ]
        Ts = [np.eye(4)]
        for p in params:
            Ts.append(Ts[-1] @ self.dh(*p))
        return Ts

    def joint_positions(self, q):
        """(5, 3) array of [base, shoulder, elbow, wrist, tcp] XYZ positions for 3D plotting."""
        return np.array([T[:3, 3] for T in self.transforms(q)])

    # ---------- forward kinematics ----------
    def fk(self, q):
        return self.transforms(q)[-1]

    def tcp_position(self, q):
        return self.fk(q)[:3, 3]

    @staticmethod
    def tool_pitch(q):
        return float(q[1] + q[2] + q[3])

    # ---------- inverse kinematics (analytic) ----------
    def ik(self, p, phi, elbow_down=True):
        """Return (q, message); q is None if unreachable / outside limits."""
        g = self.g
        x, y, z = p
        r = np.hypot(x, y)
        if r < 0.01:
            return None, "Target too close to base axis (pan singularity)"

        q1 = np.arctan2(y, x) - np.pi / 2.0
        q1 = (q1 + np.pi) % (2.0 * np.pi) - np.pi            # wrap to [-pi, pi)
        rw = r - g.a1 - g.a4 * np.cos(phi)
        zw = g.d1 - z - g.a4 * np.sin(phi)

        c3 = (rw**2 + zw**2 - g.a2**2 - g.a3**2) / (2.0 * g.a2 * g.a3)
        if abs(c3) > 1.0 + 1e-6:
            return None, "Target out of reach"
        c3 = float(np.clip(c3, -1.0, 1.0))

        # Try preferred elbow branch first, then the alternate branch
        signs = (1.0, -1.0) if elbow_down else (-1.0, 1.0)
        for sign in signs:
            s3 = sign * np.sqrt(1.0 - c3**2)
            q3 = np.arctan2(s3, c3)
            q2 = np.arctan2(zw, rw) - np.arctan2(g.a3 * s3, g.a2 + g.a3 * c3)
            q = np.array([q1, q2, q3, phi - q2 - q3], dtype=float)
            if not self.out_of_limits(q):
                return q, "ok"

        return None, "Solution violates joint limits"

    def out_of_limits(self, q):
        return bool(np.any(q < self.limits[:, 0]) or np.any(q > self.limits[:, 1]))

    # ---------- velocity kinematics ----------
    def jacobian(self, q):
        """6x4 geometric Jacobian [v; w] in the base frame (v in m/rad)."""
        g = self.g
        t1 = q[0] + np.pi / 2.0
        c1, s1 = np.cos(t1), np.sin(t1)
        c2, s2 = np.cos(q[1]), np.sin(q[1])
        c23, s23 = np.cos(q[1] + q[2]), np.sin(q[1] + q[2])
        c234, s234 = np.cos(q[1] + q[2] + q[3]), np.sin(q[1] + q[2] + q[3])

        R = g.a1 + g.a2 * c2 + g.a3 * c23 + g.a4 * c234
        L2 = g.a2 * s2 + g.a3 * s23 + g.a4 * s234
        L3 = g.a3 * s23 + g.a4 * s234
        L4 = g.a4 * s234
        M2 = g.a2 * c2 + g.a3 * c23 + g.a4 * c234
        M3 = g.a3 * c23 + g.a4 * c234
        M4 = g.a4 * c234

        return np.array([
            [-s1 * R, -c1 * L2, -c1 * L3, -c1 * L4],
            [ c1 * R, -s1 * L2, -s1 * L3, -s1 * L4],
            [    0.0,      -M2,      -M3,      -M4],
            [    0.0,      -s1,      -s1,      -s1],
            [    0.0,       c1,       c1,       c1],
            [    1.0,      0.0,      0.0,      0.0],
        ])

    def linear_jacobian(self, q):
        return self.jacobian(q)[:3]

    def task_jacobian(self, q):
        """4x4 Jacobian for [x, y, z, pitch] (pitch row weighted by pitch_w)."""
        return np.vstack([self.linear_jacobian(q), self.pitch_w * np.array([0.0, 1.0, 1.0, 1.0])])

    def forward_velocity(self, q, q_dot):
        """TCP velocity [vx, vy, vz, vpitch] (m/s, rad/s) from joint rates q_dot (rad/s)."""
        q_dot = np.asarray(q_dot, dtype=float)
        return np.append(self.linear_jacobian(q) @ q_dot, q_dot[1:].sum())

    def resolved_rate(self, q, v):
        """Joint rates (rad/s) for TCP velocity v=[vx,vy,vz,vpitch]; damped least squares."""
        status = self.singularity(q)
        if status.level == "stop":
            return np.zeros(4), status

        J = self.task_jacobian(q)
        v_scaled = np.array(v, dtype=float)
        v_scaled[3] *= self.pitch_w
        q_dot = J.T @ np.linalg.solve(J @ J.T + (self.damping**2) * np.eye(4), v_scaled)
        return q_dot, status

    def manipulability(self, q):
        J = self.linear_jacobian(q)
        return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

    def singularity(self, q):
        s = np.linalg.svd(self.task_jacobian(q), compute_uv=False)
        idx = float(s[-1] / s[0]) if s[0] > 1e-9 else 0.0
        if idx < self.stop:
            return SingularityStatus(idx, "stop", "SINGULARITY - motion blocked")
        if idx < self.warn:
            return SingularityStatus(idx, "warn", "Near singularity - slow down / change pose")
        return SingularityStatus(idx, "ok", "OK")


if __name__ == "__main__":
    print("[Kinematics] Running self-test...")
    kin = SO101Kinematics()

    q_deg = np.array([20.0, -100.0, 60.0, 20.0])      # inside the joint limits
    q = np.radians(q_deg)
    tcp, phi = kin.tcp_position(q), kin.tool_pitch(q)
    print(f"FK position (cm): {(tcp * 100).round(4)}, pitch: {np.degrees(phi):.2f} deg")

    q_sol, msg = kin.ik(tcp, phi, elbow_down=True)
    assert q_sol is not None, f"IK failed: {msg}"
    assert np.allclose(q_sol, q, atol=1e-9), "IK did not reproduce the joint angles"
    print(f"IK angles (deg): {np.degrees(q_sol).round(4)}")

    eps = 1e-6                                   # Jacobian vs numerical derivative
    Jn = np.column_stack([(kin.tcp_position(q + eps * e) - kin.tcp_position(q - eps * e)) / (2 * eps)
                          for e in np.eye(4)])
    assert np.allclose(kin.linear_jacobian(q), Jn, atol=1e-6), "Jacobian mismatch"
    print(f"Jacobian (6x4):\n{kin.jacobian(q).round(4)}")

    q_dot, status = kin.resolved_rate(q, [0.02, 0.0, -0.01, 0.0])
    print(f"Singularity: {status.level} (index={status.index:.3f}), "
          f"verified TCP vel: {kin.forward_velocity(q, q_dot).round(4)}")
    assert not kin.out_of_limits(kin.home), "home pose outside joint limits"
    assert not kin.out_of_limits(kin.calib), "calibration pose outside joint limits"
    print("[Kinematics] Self-test passed successfully!")
