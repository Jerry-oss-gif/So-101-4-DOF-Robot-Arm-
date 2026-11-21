# SO-101 Robot Studio (real_test)

Run everything from this folder, e.g. `python main.py`.

## Files
| file | purpose |
|---|---|
| main.py | start the GUI (`python main.py`, or `python main.py --real --port COM3`) |
| gui.py / controller.py | window, safety, motion |
| kinematics.py | DH model, IK, Jacobian, joint limits, home pose, calibration pose |
| hardware.py | servo <-> angle conversion (true 360 deg = 4096 ticks), calibration store |
| calibrate_home.py | command-line calibration (same pose as the GUI "Calibrate" dialog) |
| diagnose.py | live table: real arm vs model |
| calibration.json | sign + zero_deg per joint (written by calibrate_home.py / the GUI) |

## Calibrate (do this whenever a horn was moved or numbers look wrong)
1. Torque OFF. Hold the arm UPRIGHT: upper arm vertical, forearm horizontal forward, wrist level, pan straight ahead.
2. `python calibrate_home.py COM3` (or GUI -> Calibrate -> Apply & Save). Nudge each joint when asked (sets the direction).
3. `python diagnose.py COM3` with the arm upright: `raw` ~ `expect`, every `err` ~ 0.
4. Move one joint by hand: the model must follow in the same direction and by the same angle.
5. Only then switch torque ON and press Home.

Positive direction: pan counter-clockwise (seen from above); shoulder / elbow / wrist DOWN.
