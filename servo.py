"""Quick baud/ID check on one port.  Usage: python servo.py COM5"""
import sys
from scservo_sdk import PortHandler, sms_sts

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"   # pass your port as an argument

for baud in (1_000_000, 500_000, 250_000, 115_200, 57_600):
    ph = PortHandler(PORT)
    if not ph.openPort() or not ph.setBaudRate(baud):
        print(f"{baud}: cannot open port {PORT}")
        continue
    pk = sms_sts(ph)
    found = [i for i in range(0, 20) if pk.ping(i)[1] == 0]
    print(f"baud {baud}: found IDs {found}")
    ph.closePort()
