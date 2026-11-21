"""Hardware layer: 4 x Feetech STS3215 servos (IDs 1-4) behind a common RobotInterface."""
import json
import os
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, asdict
from typing import ClassVar
import numpy as np

ADDR_MODE, ADDR_TORQUE = 33, 40

@dataclass(frozen=True)
class ServoRange:
    """Linear tick <-> degree map: 0 ticks = min_deg, ticks/2 = centre (0 deg), `ticks` = max_deg."""
    ticks: int = 4096          # STS3215 position resolution (valid values 0..ticks-1)
    min_deg: float = -180.0
    max_deg: float = 180.0

    @property
    def span_deg(self):
        return self.max_deg - self.min_deg

    def ticks_to_deg(self, ticks):
        return float(ticks) / self.ticks * self.span_deg + self.min_deg

    def deg_to_ticks(self, deg):
        t = round((deg - self.min_deg) / self.span_deg * self.ticks)
        return int(np.clip(t, 0, self.ticks - 1))

    def contains(self, deg):
        return self.min_deg - 1e-9 <= deg <= self.max_deg + 1e-9


SERVO_RANGE = ServoRange()
TICKS = SERVO_RANGE.ticks
RANGE_DEG = SERVO_RANGE.max_deg


def ticks_to_deg(ticks):
    """Raw servo ticks -> servo degrees in [-180, +180)."""
    return SERVO_RANGE.ticks_to_deg(ticks)


def deg_to_ticks(deg):
    """Servo degrees -> raw ticks, clamped to the valid servo range 0..4095."""
    return SERVO_RANGE.deg_to_ticks(deg)


@dataclass
class JointConfig:
    servo_id: int
    name: str
    sign: int = 1            # +1 / -1 : direction of the servo relative to the kinematic model
    zero_deg: float = 0.0    # servo angle (-180..+180 deg) when model angle q = 0
    RANGE: ClassVar[ServoRange] = SERVO_RANGE   # shared tick/degree map (not saved to JSON)

    @classmethod
    def from_dict(cls, d):
        d = dict(d)
        if "zero_ticks" in d:  # migrate old calibration files
            d["zero_deg"] = round(cls.RANGE.ticks_to_deg(d.pop("zero_ticks")), 2)
        return cls(**d)

    def servo_deg(self, q):
        """Servo angle (deg, wrapped into -180..+180) needed for model angle q (rad)."""
        d = self.zero_deg + self.sign * float(np.degrees(q))
        return (d + 180.0) % 360.0 - 180.0

    def to_ticks(self, q):
        return self.RANGE.deg_to_ticks(self.servo_deg(q))

    def from_ticks(self, ticks):
        d = self.RANGE.ticks_to_deg(ticks) - self.zero_deg
        d = (d + 180.0) % 360.0 - 180.0          # shortest way round the encoder wrap (tick 0/4095)
        return float(np.radians(self.sign * d))

    def calibrate(self, raw_ticks, q_ref):
        """Joint is physically at model angle q_ref (rad) with encoder reading raw_ticks."""
        z = self.RANGE.ticks_to_deg(raw_ticks) - self.sign * float(np.degrees(q_ref))
        self.zero_deg = round((z + 180.0) % 360.0 - 180.0, 2)

    def servo_span_deg(self, limits_rad, ref_rad=0.0):
        """Servo-degree interval covered by the model limits (lo, hi) in rad,
        measured outward from the reference pose ref_rad (e.g. the home pose).
        At ref_rad the servo is at its real encoder angle (-180..+180),
        so the interval only leaves -180..+180 if the joint really crosses the wrap point."""
        lo, hi = np.degrees(limits_rad)
        ref = float(np.degrees(ref_rad))
        s_ref = (self.zero_deg + self.sign * ref + 180.0) % 360.0 - 180.0   # servo angle at home
        a = s_ref + self.sign * (lo - ref)
        b = s_ref + self.sign * (hi - ref)
        return min(a, b), max(a, b)

    def fits_range(self, limits_rad, ref_rad=0.0):
        lo, hi = self.servo_span_deg(limits_rad, ref_rad)
        return self.RANGE.contains(lo) and self.RANGE.contains(hi)


class CalibrationStore:
    def __init__(self, path="calibration.json"):
        if not os.path.isabs(path):   # same file for GUI / calibrate_home / diagnose, whatever the working directory
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), path)
        self.path = path
        self.warning = ""             # non-empty when load() had to fall back to the uncalibrated defaults

    @staticmethod
    def default_joints():
        """UNCALIBRATED placeholder (zero_deg = 0). Never trust it on the real arm."""
        names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
        return [JointConfig(i + 1, n) for i, n in enumerate(names)]

    def load(self):
        self.warning = ""
        if not os.path.exists(self.path):
            self.warning = f"Calibration file not found:\n{self.path}\nThe arm is NOT calibrated."
        else:
            try:
                with open(self.path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if isinstance(data, list) and len(data) > 0:
                    return [JointConfig.from_dict(d) for d in data]
                self.warning = f"Calibration file is empty or malformed:\n{self.path}\nThe arm is NOT calibrated."
            except Exception as e:
                self.warning = f"Cannot read calibration file:\n{self.path}\n({e})\nThe arm is NOT calibrated."
        return self.default_joints()

    def save(self, joints):
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump([asdict(j) for j in joints], f, indent=2)

class RobotInterface(ABC):
    real = False

    @abstractmethod
    def connect(self): ...
    @abstractmethod
    def disconnect(self): ...
    @abstractmethod
    def read_q(self): ...
    @abstractmethod
    def write_q(self, q): ...
    @abstractmethod
    def set_torque(self, on): ...

class SimRobot(RobotInterface):
    """Perfect virtual arm for testing the GUI without hardware."""
    def __init__(self, q0):
        self.q = np.array(q0, float)

    def connect(self): pass
    def disconnect(self): pass
    def read_q(self): return self.q.copy()
    def write_q(self, q): self.q = np.array(q, float)
    def set_torque(self, on): pass

    def read_raw(self):
        """Simulate raw encoder ticks corresponding to simulated joint angles."""
        return [deg_to_ticks(np.degrees(qi)) for qi in self.q]

class FeetechRobot(RobotInterface):
    real = True

    def __init__(self, port, joints, baud=1_000_000, speed=2000, acc=50):
        self.port, self.joints, self.baud, self.speed, self.acc = port, joints, baud, speed, acc
        self.ph = self.pk = None
        self.last_raw = None          # latest raw ticks read from the bus (for the GUI, avoids extra bus traffic)

    # ---- connection ----
    def connect(self):
        self.disconnect()
        try:
            from scservo_sdk import PortHandler, sms_sts  # pip install ftservo-python-sdk
        except ImportError as e:
            raise ImportError(
                "Feetech SDK with sms_sts support is required. "
                "Please install ftservo-python-sdk: pip install ftservo-python-sdk"
            ) from e

        self.ph = PortHandler(self.port)
        if not self.ph.openPort():
            self.disconnect()
            raise IOError(f"Cannot open serial port {self.port}")
        if not self.ph.setBaudRate(self.baud):
            self.disconnect()
            raise IOError(f"Cannot set baud rate {self.baud} on {self.port}")
        self.pk = sms_sts(self.ph)
        time.sleep(0.2)  # let the USB adapter settle after opening the port

        def responds(sid):
            for _ in range(3):  # retry to ride out a dropped packet
                if self.pk.ping(sid)[1] == 0:
                    return True
                time.sleep(0.05)
            return False

        missing = [j.servo_id for j in self.joints if not responds(j.servo_id)]
        if missing:
            self.disconnect()
            raise IOError(
                f"No response from servo ID(s) {missing} on {self.port} @ {self.baud} baud. "
                "Check: servo power supply is ON, correct COM port, adapter jumper on USB channel, "
                "baud rate (run scan_servo.py), and unique servo IDs (run set_servo_id.py)."
            )
        for j in self.joints:  # position (servo) mode
            self.pk.write1ByteTxRx(j.servo_id, ADDR_MODE, 0)

    def disconnect(self):
        try:
            if self.ph:
                self.ph.closePort()
        except Exception:
            pass
        finally:
            self.ph = self.pk = None

    # ---- I/O ----
    def read_raw(self):
        if not self.pk:
            raise IOError("Robot not connected")
        out = []
        for j in self.joints:
            pos, res, _ = self.pk.ReadPos(j.servo_id)
            if res != 0:
                raise IOError(f"Read failed on servo {j.servo_id}")
            out.append(int(pos))
        self.last_raw = out
        return out

    def read_q(self):
        return np.array([j.from_ticks(t) for j, t in zip(self.joints, self.read_raw())], dtype=float)

    def write_q(self, q):
        if not self.pk:
            raise IOError("Robot not connected")
        for j, qi in zip(self.joints, q):   # never silently clamp: the arm would stop somewhere else than the GUI shows
            if not j.RANGE.contains(j.servo_deg(qi)):
                raise IOError(f"{j.name}: {np.degrees(qi):.1f} deg is outside the servo range (encoder wrap at tick 0/4095) "
                              "- re-seat the horn / recalibrate")
        self.pk.groupSyncWrite.clearParam()
        try:
            for j, qi in zip(self.joints, q):
                if not self.pk.SyncWritePosEx(j.servo_id, j.to_ticks(qi), self.speed, self.acc):
                    raise IOError(f"SyncWrite addParam failed for servo {j.servo_id}")
            res = self.pk.groupSyncWrite.txPacket()
            if res != 0:
                raise IOError(f"SyncWrite transmit failed with code {res}")
        finally:
            self.pk.groupSyncWrite.clearParam()

    def set_torque(self, on):
        if not self.pk:
            return
        if on:  # hold current pose first -> no jump when torque enables
            try:
                self.write_q(self.read_q())
            except Exception:
                pass
        for j in self.joints:
            self.pk.write1ByteTxRx(j.servo_id, ADDR_TORQUE, 1 if on else 0)

    # ---- calibration ----
    def calibrate_reference(self, q_ref, limits=None):
        """The arm is physically in pose q_ref (rad) right now -> store zero_deg for each joint.
        Returns names of joints whose model range (limits, rad) would cross the servo wrap point (tick 0/4095)."""
        for j, raw, q in zip(self.joints, self.read_raw(), q_ref):
            j.calibrate(raw, q)
        if limits is None:
            return []
        return [j.name for j, lim in zip(self.joints, limits) if not j.fits_range(lim)]

if __name__ == "__main__":
    print("[Hardware] Running self-test...")
    store = CalibrationStore()
    joints = store.load()
    print(f"Loaded {len(joints)} joints: {[j.name for j in joints]}")
    sim = SimRobot([0, 0, 0, 0])
    raw = sim.read_raw()
    print(f"SimRobot raw encoder ticks: {raw}")
    print("[Hardware] Self-test passed successfully!")

