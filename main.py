#!/usr/bin/env python3
"""SO-101 4-DOF Robot Studio - entry point.

Examples
  python main.py                          # simulation, no hardware
  python main.py --real --port COM3       # connect to the 4 Feetech servos on start-up
  python main.py --port /dev/ttyACM0      # pre-select the port (connect from the GUI)
"""
import argparse
import sys

def parse_args():
    p = argparse.ArgumentParser(description="SO-101 4-DOF robot controller with 3D visualizer")
    p.add_argument("--real", action="store_true", help="use the real Feetech STS3215 arm and connect on start-up")
    p.add_argument("--port", default="COM3", help="serial port of the servo bus board (default: COM3)")
    p.add_argument("--calibration", default="calibration.json", help="calibration file (default: calibration.json)")
    return p.parse_args()



def main():
    args = parse_args()
    import importlib.util                           # imported late so --help works without a display
    if importlib.util.find_spec("so101"):           # package layout: so101/ folder next to main.py
        from so101.gui import RobotStudioApp
    else:                                           # flat layout: gui.py, kinematics.py ... next to main.py
        from gui import RobotStudioApp

    app = RobotStudioApp(calibration=args.calibration, port=args.port,
                         mode="Feetech STS3215" if args.real else "Simulation")
    if args.real:
        app.after(300, app.connect)                 # connect once the window is up (errors show in a dialog)
    app.mainloop()


if __name__ == "__main__":
    main()
