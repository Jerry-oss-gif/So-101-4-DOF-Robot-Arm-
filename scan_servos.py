"""Find the Feetech servo bus: lists serial ports, then scans baud rates and IDs.

  python scan_servos.py                 # every serial port, IDs 0-19
  python scan_servos.py COM5            # one port
  python scan_servos.py COM5 --full     # IDs 0-253 (use if the servo ID is unknown)
"""
import sys
from scservo_sdk import PortHandler, sms_sts

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pip install pyserial")

BAUDS = (1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400)  # all STS3215 baud codes


def raw_ping(port, baud, sid=1):
    """Send a ping packet with pyserial only (no SDK) and return the bytes that come back."""
    with serial.Serial(port, baud, timeout=0.05) as s:
        s.reset_input_buffer()
        pkt = bytes([0xFF, 0xFF, sid, 0x02, 0x01, (~(sid + 3)) & 0xFF])
        s.write(pkt)
        return pkt, s.read(16)


def interpret(sent, got):
    if not got:
        return "nothing came back"
    if got == sent:
        return "got back exactly what was sent (TX/RX looped or bus adapter echo)"
    body = got.lstrip(b"\xff")
    if len(body) >= 4 and body[1] == 2 and (~(body[0] + body[1] + body[2])) & 0xFF == body[3]:
        if got[:2] == b"\xff\xff":
            return f"servo ID {body[0]} replied"
        return (f"servo ID {body[0]} replied, header damaged -> if several servos are connected they probably "
                "ALL have this ID and answer at once (give each a unique ID)")
    return "garbled reply (wrong baud, or several servos with the same ID answering at once)"


def scan_port(port, ids):
    hit = False
    for baud in BAUDS:
        ph = PortHandler(port)
        if not ph.openPort() or not ph.setBaudRate(baud):
            print(f"  {baud:>8}: port/baud rate not accepted (port busy, or driver lacks this baud)")
            continue
        pk = sms_sts(ph)
        found = [i for i in ids if pk.ping(i)[1] == 0]
        ph.closePort()
        print(f"  {baud:>8}: found IDs {found}")
        hit |= bool(found)
    return hit


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    ids = range(0, 254) if "--full" in sys.argv else range(0, 20)
    if "--full" in sys.argv:
        print("Full scan (IDs 0-253 x 8 baud rates) takes about a minute per port...")

    print("Serial ports on this PC:")
    ports = list(list_ports.comports())
    for p in ports:
        vid = f"  VID:PID={p.vid:04X}:{p.pid:04X}" if p.vid else ""
        print(f"  {p.device:8} {p.description}{vid}")
    if not ports:
        sys.exit("  none found -> USB cable / driver (CH340, CP210x or FTDI) problem")

    targets = args or [p.device for p in ports]
    any_hit = False
    for port in targets:
        print(f"\n== {port} ==")
        any_hit |= scan_port(port, ids)
        sent, got = raw_ping(port, 1_000_000)
        verdict = interpret(sent, got)
        print(f"  raw ping ID1 @1M: sent {sent.hex(' ')} | received {got.hex(' ') or '-'} -> {verdict}")

    if not any_hit:
        print("""
No servo answered. Check, in this order:
 1. POWER: the servos need their own supply (about 6-7.4 V for the 7.4 V STS3215, 12 V for the 12 V version)
    plugged into the bus adapter. USB only powers the adapter chip, not the motors.
 2. PORT: pick the entry above that looks like CH340 / CP210x / FTDI / 'USB Serial'. A Bluetooth COM port
    also 'opens' fine but never answers.
 3. ADAPTER JUMPER (Waveshare Bus Servo Adapter): jumper cap must be on channel A (USB), not B.
 4. CABLE: 3-pin servo cable fully seated at both ends; try the other socket on the servo.
 5. Other programs (the GUI, LeRobot, Feetech software) must be closed.
 6. Unknown ID / baud: run with --full, with only ONE servo connected.
 7. Several servos connected but one works alone? They all have ID 1 -> set unique IDs (set_servo_id.py).""")


if __name__ == "__main__":
    main()
