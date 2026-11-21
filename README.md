(https://github.com/user-attachments/files/35216768/README.1.md)
# SO-101 Robot Studio

A simple Python program to control a 4-joint **SO-101 robot arm** with **Feetech STS3215** servos.
It has a window (GUI) with a live 3D view of the arm. You can also test everything in **simulation** without any hardware.

## Features

- 3D view of the arm that moves with the real arm
- Move each joint with sliders (forward kinematics)
- Type a target X, Y, Z and pitch, and the arm moves there (inverse kinematics)
- Jog the tip in X, Y, Z by holding a button
- Adjustable speed
- Singularity warning (tells you when the arm is in a bad pose)
- Safety: emergency stop, joint limits, lost-connection stop, and "arm is blocked" stop
- Simple calibration, so the 3D model and the real arm match
- Simulation mode (no robot needed)

## What you need

- Python 3.10 or newer
- A PC with a display (the GUI uses Tkinter)
- For the real arm:
  - 4 x Feetech STS3215 servos with IDs 1, 2, 3, 4
  - A USB servo adapter (for example Waveshare Bus Servo Adapter)
  - A power supply for the servos (USB alone does **not** power the motors)

## Install

```bash
git clone github.com/Jerry-oss-gif/So-101-4-DOF-Robot-Arm-
cd YOUR-REPO
pip install -r requirements.txt
```

## Quick start

Simulation (no hardware):

```bash
python main.py
```
<img width="2880" height="1800" alt="Screenshot 2026-11-22 004845" src="https://github.com/user-attachments/assets/f2f8c4af-8e02-4704-b164-f10d0b2e480f" />
Example for UI 

Real arm (connects when the window opens):

```bash
python main.py --real --port COM3
```

On Linux or Mac, use a port like `/dev/ttyACM0` or `/dev/ttyUSB0`.

| Option | Meaning | Default |
|---|---|---|
| `--real` | Use the real arm and connect at start | off (simulation) |
| `--port` | Serial port of the servo adapter | `COM3` |
| `--calibration` | Calibration file to use | `calibration.json` |

## Setup order for a new arm

1. `scan_servos.py` → find the servos
2. `set_servo_id.py` → give each servo a unique ID (1, 2, 3, 4)
3. Calibrate (GUI **Calibrate** button or `calibrate_home.py`)
4. `diagnose.py` → check that the real arm and the 3D model match
5. `main.py --real` → run the GUI, turn torque ON, press Home

## How to use each file

### Files you run

**`main.py`** starts the program (the window with the 3D arm).

```bash
python main.py                        # simulation, no robot
python main.py --real --port COM3     # real arm
```

In the window: choose the mode and port at the top, press **Connect**, then **Calibrate** (first time), then tick **Torque ON** and press **Home**.

**`scan_servos.py`** finds your servos. Use it first if the arm does not connect.

```bash
python scan_servos.py              # checks all ports
python scan_servos.py COM5         # one port
python scan_servos.py COM5 --full  # checks IDs 0-253 (slow)
```

It prints which baud rate and IDs answered, and gives tips if nothing answers.

**`servo.py`** is a quick, smaller version of the scan. It checks 5 baud rates and IDs 0-19.

```bash
python servo.py COM5
```

**`set_servo_id.py`** gives a servo a new ID. Connect **only one servo** at a time. New servos are usually ID 1, and this project needs IDs 1, 2, 3, 4.

```bash
python set_servo_id.py COM5 1 2    # port, old ID, new ID
```

**`calibrate_home.py`** calibrates from the command line (torque off, you move the arm by hand).

```bash
python calibrate_home.py COM3
```

You can also calibrate in the GUI with the **Calibrate** button.

**`diagnose.py`** shows a live table: real arm vs 3D model. Use it after calibration.

```bash
python diagnose.py COM3          # check the calibration pose
python diagnose.py COM3 home     # check the home pose
```

`err` should be close to 0. Press **Ctrl+C** to quit.

**`scanposition.py`** prints the raw position of each servo. Edit the `PORT = "COM3"` line inside the file, then run:

```bash
python scanposition.py
```

Use it only to read positions. Do not copy its numbers into `zero_deg` by hand. Let the Calibrate step do that.

**`hardware.py`** and **`kinematics.py`** have a built-in self-test, so you can check them without a robot:

```bash
python hardware.py
python kinematics.py
```

### Files you do not run

| File | What it does |
|---|---|
| `gui.py` | The window, sliders, buttons, and 3D view. `main.py` loads it. |
| `controller.py` | Moves the arm, runs jogging, and handles safety (emergency stop, joint limits, lost connection). |
| `kinematics.py` | The math of the arm (forward and inverse kinematics, joint limits, home pose). |
| `hardware.py` | Talks to the servos, converts ticks to angles, loads and saves `calibration.json`. |
| `calibration.json` | Saved calibration (`sign` and `zero_deg` for each joint). |
| `__init__.py` | Lets you import the files as a package (`so101`). You can ignore it in a flat folder. |
| `requirements.txt` | Python packages to install. |

## Calibration (important!)

Do this the first time, and again whenever a servo horn was moved or the numbers look wrong.

1. Turn **torque OFF**.
2. Hold the arm in the **calibration pose**: the same pose as `calib_deg` in `kinematics.py`
   (pan straight ahead, and the joint angles `0, -143, 99, 67` degrees).
3. Run `python calibrate_home.py COM3`, or use the GUI: **Calibrate → Apply & Save**.
   Move each joint a little when asked. This sets the direction (`sign`) of each joint.
4. Check it with `python diagnose.py COM3`.
   The `raw` value should be close to `expect`, and every `err` should be close to 0.
5. Move one joint by hand. The 3D model must move in the **same direction** and by the **same angle**.
6. Only then turn **torque ON** and press **Home**.

**Positive direction:** pan = counter-clockwise (seen from above). Shoulder, elbow, wrist = down.

## Feetech servo software (official)

To set up or test a servo with a window program (change ID, baud rate, check the servo, update firmware), download the official Feetech software here:
[https://www.feetechrc.com/software.html](https://www.feetechrc.com/software.html)

<img width="2446" height="1570" alt="Screenshot 2026-11-22 003532" src="https://github.com/user-attachments/assets/6572dbd8-14bc-4f8b-93f4-479ffec519ad" />
Download the FD 1.9....etc

Close this program before you use it, because only one program can use the port at a time.

## Safety tips

- Always **support the arm** before turning torque off or closing the program. It can fall.
- Check that the 3D model matches the real arm **before** turning torque ON.
- Keep your hand near the **EMERGENCY STOP** button the first time you run a new calibration.
- If a joint shows `[NEAR WRAP!]`, the servo is close to its 0/4095 limit. Re-mount the horn one spline step and calibrate again.

## Troubleshooting

If no servo answers:

1. Is the servo power supply ON?
2. Is it the right COM port? (Look for CH340, CP210x, FTDI, or "USB Serial".)
3. On a Waveshare adapter, the jumper must be on channel **A** (USB).
4. Is the 3-pin cable fully pushed in?
5. Close other programs that use the port (this GUI, LeRobot, Feetech software).
6. Do all servos have the same ID? Give each one a unique ID with `set_servo_id.py`.
