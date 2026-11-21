from hardware import CalibrationStore, FeetechRobot, ticks_to_deg

PORT = "COM3"  # Change to your serial port (e.g., "/dev/ttyUSB0" or "/dev/tty.usbmodem...")

store = CalibrationStore()
joints = store.load()
robot = FeetechRobot(port=PORT, joints=joints)

try:
    robot.connect()
    robot.set_torque(False)  # Disable torque so you can move the arm freely by hand

    raw_ticks = robot.read_raw()
    degs = [round(ticks_to_deg(t), 2) for t in raw_ticks]
    for joint, ticks, d in zip(joints, raw_ticks, degs):
        print(f"ID {joint.servo_id} ({joint.name}): {ticks} ticks = {d} deg")

    print("\nCopy-paste list for zero_deg:", degs)
finally:
    robot.disconnect()