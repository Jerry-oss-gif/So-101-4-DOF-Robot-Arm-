"""Assign a unique ID to ONE connected STS3215 servo.
Connect only the servo you want to change, power it, close the GUI.

Usage:  python set_servo_id.py COM5 <current_id> <new_id> [baud]
Example (new servos ship as ID 1):  python set_servo_id.py COM5 1 2
"""
import sys
import time
from scservo_sdk import PortHandler, sms_sts

ADDR_ID, ADDR_LOCK = 5, 55

if len(sys.argv) < 4:
    sys.exit(__doc__)
port, old, new = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
baud = int(sys.argv[4]) if len(sys.argv) > 4 else 1_000_000
if not (0 <= new <= 253):
    sys.exit("New ID must be 0-253")

ph = PortHandler(port)
if not ph.openPort() or not ph.setBaudRate(baud):
    sys.exit(f"Cannot open {port} @ {baud}")
pk = sms_sts(ph)
time.sleep(0.2)

if pk.ping(old)[1] != 0:
    ph.closePort()
    sys.exit(f"No servo answered at ID {old}. Check power, port, baud.")

pk.write1ByteTxRx(old, ADDR_LOCK, 0)        # unlock EEPROM
pk.write1ByteTxRx(old, ADDR_ID, new)        # write new ID
time.sleep(0.1)
pk.write1ByteTxRx(new, ADDR_LOCK, 1)        # lock EEPROM
time.sleep(0.1)

ok = pk.ping(new)[1] == 0
print(f"ID {old} -> {new}: {'OK' if ok else 'FAILED (servo does not answer at new ID)'}")
ph.closePort()
