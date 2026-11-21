"""MotionController: joint/jog/move-to-target logic, safety, and robot I/O. No GUI code here."""
import numpy as np


class MotionController:
    DT = 0.02             # control period (s) - call update() at this rate
    READ_EVERY = 5        # read servo feedback every N ticks (10 Hz)
    MAX_COMM_ERR = 5      # consecutive I/O errors before E-STOP
    FOLLOW_ERR = np.radians(25)   # |commanded - actual| limit
    FOLLOW_TICKS = 15     # ... for this many reads (1.5 s) -> E-STOP

    def __init__(self, kin, robot):
        self.kin = kin
        self.speed_mm_s = 30.0          # TCP speed for jog / move
        self.max_joint_dps = 90.0       # slew-rate limit per joint (deg/s)
        self.attach(robot)

    # ---------------- lifecycle ----------------
    def attach(self, robot):
        self.robot = robot
        self.torque, self.estopped, self.mode, self.message = False, False, "idle", ""
        self.jog = np.zeros(3)
        self.q_fb = robot.read_q()
        self.q_cmd, self.q_des, self.qd = self.q_fb.copy(), self.q_fb.copy(), np.zeros(4)
        self._tick = self._comm_err = self._follow = 0
        self._io_ok = True
        if self.kin.out_of_limits(self.q_fb):
            self.message = "Actual pose outside model limits - check calibration"

    def shutdown(self):
        try:
            self.set_torque(False)
            self.robot.disconnect()
        except Exception:
            pass

    # ---------------- safety ----------------
    def _io(self, fn, *args):
        try:
            r = fn(*args); self._comm_err = 0; self._io_ok = True
            return r
        except Exception as e:
            self._io_ok = False
            self._comm_err += 1; self.message = f"Comm error: {e}"
            if self._comm_err >= self.MAX_COMM_ERR:
                self.emergency("Communication lost - E-STOP")
            return None

    def emergency(self, why="EMERGENCY STOP - press RESET to continue"):
        self.estopped, self.mode, self.message = True, "idle", why
        self.jog[:] = 0
        try: self.robot.set_torque(False)
        except Exception: pass
        self.torque = False

    def reset_estop(self):
        self.estopped, self.message, self._comm_err, self._follow = False, "", 0, 0
        q = self._io(self.robot.read_q)
        if q is not None:
            self.q_fb, self.q_cmd, self.q_des = q, q.copy(), q.copy()

    def stop(self, why=""):
        self.mode = "idle"; self.jog[:] = 0
        self.q_des = self.q_cmd.copy()
        if why: self.message = why

    def set_torque(self, on):
        if self.estopped and on:
            self.message = "E-STOP active - reset first"; return False
        if on:
            q = self._io(self.robot.read_q)
            if q is None: return False
            self.q_fb, self.q_cmd, self.q_des = q, q.copy(), q.copy()
        self._io(self.robot.set_torque, on)
        if not self._io_ok and on:
            return False
        self.torque = on; self.mode = "idle"; self.message = ""
        return True

    # ---------------- commands ----------------
    def _ready(self):
        if self.estopped: self.message = "E-STOP active"; return False
        if not self.torque: self.message = "Enable torque first"; return False
        return True

    def set_joint_target(self, i, q):
        if not self._ready(): return
        self.mode, self.message = "idle", ""
        self.q_des = self.q_des.copy()
        self.q_des[i] = np.clip(q, *self.kin.limits[i])

    def go_home(self):
        if not self._ready():
            return
        if self.kin.out_of_limits(self.kin.home):
            self.message = "Home pose is outside joint limits"
            return
        self.mode, self.message = "idle", "Moving to home..."
        self.q_des = self.kin.home.copy()

    def start_jog(self, axis, sign):
        if not self._ready(): return
        self.jog[:] = 0; self.jog[axis] = sign; self.mode, self.message = "jog", ""

    def stop_jog(self):
        if self.mode == "jog": self.stop()

    def move_to(self, p_mm, pitch_deg):
        """Straight-line Cartesian move at speed_mm_s. Returns (ok, message)."""
        if not self._ready(): return False, self.message
        p1, phi1 = np.asarray(p_mm, float) / 1000.0, np.radians(pitch_deg)
        p0, phi0 = self.kin.tcp_position(self.q_cmd), self.kin.tool_pitch(self.q_cmd)
        for s in np.linspace(0, 1, 60):               # validate the whole path first
            q, why = self.kin.ik(p0 + s * (p1 - p0), phi0 + s * (phi1 - phi0))
            if q is None:
                self.message = f"Cannot move to target: {why}"; return False, self.message
        self._path = (p0, p1, phi0, phi1, max(np.linalg.norm(p1 - p0), 1e-3))
        self._s, self.mode, self.message = 0.0, "move", ""
        return True, "ok"

    # ---------------- cyclic update ----------------
    def update(self):
        self._tick += 1
        if self.estopped: return
        if self._tick % self.READ_EVERY == 0 or not self.robot.real:
            q = self._io(self.robot.read_q)
            if q is not None: self.q_fb = q
        if self.estopped: return

        prev = self.q_cmd.copy()
        if not self.torque:                         # follow the arm when moved by hand
            self.q_cmd, self.q_des = self.q_fb.copy(), self.q_fb.copy()
        else:
            if self.mode == "jog": self._step_jog()
            elif self.mode == "move": self._step_move()
            step = np.radians(self.max_joint_dps) * self.DT
            self.q_cmd = self.q_cmd + np.clip(self.q_des - self.q_cmd, -step, step)
            if np.any(self.q_cmd != prev) or self._tick % 25 == 0:
                self._io(self.robot.write_q, self.q_cmd)
            self._check_following()
        self.qd = 0.7 * self.qd + 0.3 * (self.q_cmd - prev) / self.DT

    def _step_jog(self):
        v = np.append(self.jog * self.speed_mm_s / 1000.0, 0.0)
        res = self.kin.resolved_rate(self.q_cmd, v)
        q_dot = res[0] if isinstance(res, tuple) else res
        qn = self.q_cmd + q_dot * self.DT
        if self.kin.out_of_limits(qn):
            return self.stop("Joint limit reached - cannot move further")
        old, new = self.kin.singularity(self.q_cmd).index, self.kin.singularity(qn).index
        if new < self.kin.stop and new < old:
            return self.stop("Singularity - jog blocked")
        self.q_des = qn

    def _step_move(self):
        p0, p1, phi0, phi1, L = self._path
        self._s = min(1.0, self._s + self.speed_mm_s / 1000.0 * self.DT / L)
        q, why = self.kin.ik(p0 + self._s * (p1 - p0), phi0 + self._s * (phi1 - phi0))
        if q is None: return self.stop(why)
        if self.kin.singularity(q).index < self.kin.stop: return self.stop("Path passes through singularity")
        self.q_des = q
        if self._s >= 1.0: self.mode = "idle"

    def _check_following(self):
        if not self.robot.real or self._tick % self.READ_EVERY: return
        if np.max(np.abs(self.q_fb - self.q_cmd)) > self.FOLLOW_ERR:
            self._follow += 1
            if self._follow >= self.FOLLOW_TICKS:
                self.emergency("Following error - arm blocked or servo fault - E-STOP")
        else:
            self._follow = 0

    # ---------------- read-outs ----------------
    @property
    def tcp_mm(self): return self.kin.tcp_position(self.q_fb) * 1000.0
    @property
    def pitch_deg(self): return float(np.degrees(self.kin.tool_pitch(self.q_fb)))
    @property
    def singularity(self): return self.kin.singularity(self.q_cmd)
    @property
    def manipulability_mm3(self): return self.kin.manipulability(self.q_cmd) * 1e9
