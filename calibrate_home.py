"""Calibrate the REAL arm so that the 3D model and the real arm show the same pose.

  python calibrate_home.py COM3 [calibration.json]

Torque stays OFF - you move the arm by hand.
 1. Direction check: nudge each joint a little in the model's POSITIVE direction -> sets "sign".
 2. Hold the arm UPRIGHT (upper arm vertical, forearm horizontal forward, wrist level) -> stores zero_deg.
The GUI's Calibrate dialog uses the very same pose (kinematics.calib_deg), so both give the same result.
"""
import sys
import numpy as np
from hardware import CalibrationStore, FeetechRobot, SERVO_RANGE
from kinematics import SO101Kinematics

POSITIVE = {                       # model-positive direction of each joint
    "shoulder_pan":  "COUNTER-CLOCKWISE when seen from above",
    "shoulder_lift": "DOWN (upper arm tilts towards the table)",
    "elbow_flex":    "DOWN (forearm folds down relative to the upper arm)",
    "wrist_flex":    "DOWN (tool tilts towards the table)",
}

class PoseCalibrator:
    MIN_MOVE = 20   # ticks (~1.8 deg) needed to trust a direction reading

    def __init__(self, robot, kin, store, ask=input, say=print):
        self.robot, self.kin, self.store, self.ask, self.say = robot, kin, store, ask, say

    def _delta(self, a, b):                       # shortest signed tick difference (handles 0/4095 wrap)
        n = SERVO_RANGE.ticks
        return (b - a + n // 2) % n - n // 2

    def detect_signs(self):
        for i, j in enumerate(self.robot.joints):
            a = self.robot.read_raw()[i]
            self.ask(f"[{j.name}] move it a little {POSITIVE[j.name]}, then press Enter...")
            d = self._delta(a, self.robot.read_raw()[i])
            if abs(d) < self.MIN_MOVE:
                self.say(f"  no clear movement on {j.name} - keeping sign {j.sign:+d}")
            else:
                j.sign = 1 if d > 0 else -1
                self.say(f"  {j.name}: sign {j.sign:+d}")

        def calibrate_pose(self):
            deg = np.degrees(self.kin.calib)
            self.ask(f"Now hold the arm in the HOME / folded pose {deg.round(1).tolist()} deg "
                 "(J1=0, J2=-143, J3=99, J4=67), pan straight ahead. Press Enter...")
        bad = self.robot.calibrate_reference(self.kin.calib, self.kin.limits)
        self.store.save(self.robot.joints)
        for j in self.robot.joints:
            self.say(f"  {j.name}: sign {j.sign:+d}, zero_deg {j.zero_deg}")
        self.say(f"Saved to {self.store.path}")
        if bad:
            self.say(f"WARNING: {', '.join(bad)} would cross the encoder wrap point (tick 0/4095) inside its "
                     "joint limits. Re-seat that servo horn one spline step and calibrate again.")
        return bad

    def run(self):
        self.robot.set_torque(False)
        self.detect_signs()
        return 

HomeCalibrator = PoseCalibrator    # backwards-compatible name

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    store = CalibrationStore(sys.argv[2] if len(sys.argv) > 2 else "calibration.json")
    robot = FeetechRobot(port=port, joints=store.load())
    try:
        robot.connect()
        PoseCalibrator(robot, SO101Kinematics(), store).run()
        print("Done. Run diagnose.py to verify, then start the GUI.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
