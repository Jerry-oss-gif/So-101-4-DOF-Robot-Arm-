"""Live check: real arm vs model (torque OFF, you move the arm by hand).

  python diagnose.py COM3 [upright|home] [calibration.json]

Hold the arm in the chosen pose (default: upright = upper arm vertical, forearm horizontal, wrist level).
 - 'raw' must be close to 'expect' and 'err' close to 0  -> calibration is right for that pose.
 - Then move ONE joint by hand: 'model q' must change in the same direction (+ = pan counter-clockwise,
   lift/elbow/wrist DOWN) and by the same number of degrees you really turned it.
   Wrong direction -> sign.  Wrong amount -> scale.  Wrong at the pose -> zero (recalibrate).
Ctrl+C to quit.
"""
import os
import sys
import time
import numpy as np
from hardware import CalibrationStore, FeetechRobot
from kinematics import SO101Kinematics


class AlignmentMonitor:
    TOL_DEG = 3.0

    def __init__(self, robot, kin, pose="upright"):
        self.robot, self.kin, self.pose = robot, kin, pose
        self.ref = kin.home if pose == "home" else kin.calib

    def snapshot(self):
        raw = self.robot.read_raw()
        q = self.robot.read_q()
        ref = np.degrees(self.ref)
        lines = [f"Reference pose: {self.pose} {ref.round(1).tolist()} deg",
                 f"{'joint':14}{'raw':>6}{'expect':>7}{'servo°':>9}{'sign':>5}{'zero°':>9}{'model q°':>10}{'ref q°':>8}{'err°':>8}  wrap",
                 "-" * 90]
        bad = []
        for j, t, qi, r, lim, qr in zip(self.robot.joints, raw, np.degrees(q), ref, self.kin.limits, self.ref):
            err = qi - r
            risk = "" if j.fits_range(lim) else "RISK"
            if abs(err) > self.TOL_DEG:
                bad.append(j.name)
            lines.append(f"{j.name:14}{t:6d}{j.to_ticks(qr):7d}{j.RANGE.ticks_to_deg(t):9.2f}{j.sign:5d}{j.zero_deg:9.2f}"
                         f"{qi:10.2f}{r:8.1f}{err:8.2f}  {risk}")
        tcp = self.kin.tcp_position(q) * 1000
        lines.append(f"TCP mm: {tcp.round(1)}   pitch: {np.degrees(self.kin.tool_pitch(q)):.1f}°")
        lines.append(f"At {self.pose} pose: OK" if not bad else f"Not at {self.pose} pose / mis-calibrated: {', '.join(bad)}")
        return "\n".join(lines)

    def run(self, period=0.5):
        os.system("")                               # enable ANSI on Windows terminals
        while True:
            print("\033[H\033[J" + self.snapshot(), flush=True)
            time.sleep(period)


def main():
    args = sys.argv[1:]
    port = args[0] if args else "COM3"
    pose = args[1] if len(args) > 1 and args[1] in ("upright", "home") else "upright"
    cal = args[2] if len(args) > 2 else "calibration.json"
    store = CalibrationStore(cal)
    joints = store.load()
    if store.warning:
        print("WARNING:", store.warning)
    robot = FeetechRobot(port=port, joints=joints)
    try:
        robot.connect()
        robot.set_torque(False)
        AlignmentMonitor(robot, SO101Kinematics(), pose).run()
    except KeyboardInterrupt:
        pass
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
