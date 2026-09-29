"""Redo a tracker's gyroscope calibration (firmware with runtime calibration,
e.g. 0.7.3+i2cbat.2) and compare it against the old one.

    python tools/recalibrate.py            <-- it finds the CH340 port on its own
    python tools/recalibrate.py --port COM4

Steps (tracker on USB, SlimeVR CLOSED, tracker still on a table):
  1. reboot and read the stored calibration (bias points per temperature)
  2. DELCAL + reboot, then wait for the firmware to measure the bias again
  3. report the error the old calibration was making at this temperature

Do it with the tracker warm (~37-39 C, after ~20 min on the body) so the new
point lands at the temperature it actually runs at.

Saves the full log and appends a summary to firmware/calibrations.jsonl.
"""

import argparse
import json
import re
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports

FW_DIR = Path(__file__).resolve().parent.parent / "firmware"
GYRO_DPS_PER_LSB = 0.035

RE_BIAS = re.compile(r"Calibrated gyro bias at ([\d.]+)C: (-?[\d.]+) (-?[\d.]+) (-?[\d.]+)")
RE_MAC = re.compile(r"mac: ([0-9A-F:]{17})")
RE_FW = re.compile(r"SlimeVR v(\S+) starting up")


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports() if "CH340" in (p.description or "")]
    if len(ports) != 1:
        sys.exit(f"expected exactly one CH340 port, found {ports}; use --port")
    return ports[0]


class Console:
    def __init__(self, port):
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = port, 115200, 0.2
        self.s.dtr = self.s.rts = False
        try:
            self.s.open()
        except serial.SerialException as e:
            sys.exit(f"could not open {port} ({e}). Is SlimeVR closed?")
        self.log = ""

    def send(self, cmd):
        self.s.write(cmd.encode() + b"\n")

    def read_until(self, pattern, timeout):
        """Read until `pattern` shows up in the new text (or the timeout hits)."""
        start = len(self.log)
        end = time.time() + timeout
        while time.time() < end:
            chunk = self.s.read(4096)
            if chunk:
                self.log += chunk.decode("utf-8", "replace")
                if pattern and re.search(pattern, self.log[start:]):
                    return self.log[start:]
        return self.log[start:]

    def close(self):
        self.s.close()


def interp(points, temp):
    """Bias the old calibration would use at temperature `temp` (linear)."""
    if len(points) == 1:
        return points[0][1]
    (t1, b1), (t2, b2) = sorted(points)[:2]
    f = (temp - t1) / (t2 - t1)
    return [a + f * (b - a) for a, b in zip(b1, b2)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")
    port = args.port or find_port()

    con = Console(port)
    print(f"[1/3] rebooting and reading the stored calibration ({port})...")
    con.send("REBOOT")
    boot1 = con.read_until(r"sensor\(s\) configured", 25)
    fw = RE_FW.search(boot1)
    old = [(float(m[1]), [float(m[2]), float(m[3]), float(m[4])]) for m in RE_BIAS.findall(boot1)]
    if fw is None:
        con.close()
        sys.exit("never saw the tracker reboot; check the cable/port")
    if "0.7" not in fw[1] and "i2cbat" not in fw[1]:
        print(f"  aviso: firmware {fw[1]}; este script foi feito para 0.7.3+i2cbat")
    for t, b in old:
        print(f"  old point: {t:.1f} C  bias {b}")
    if not old:
        print("  no stored bias point")

    con.read_until(r"Rest calibration completed", 20)
    con.send("GET INFO")
    info = con.read_until(r"Battery voltage", 5)
    mac = RE_MAC.search(info)

    print("[2/3] wiping the calibration and rebooting. Do NOT touch the tracker (~1-2 min)...")
    con.send("DELCAL")
    con.read_until(r"Saved configuration", 5)
    con.send("REBOOT")
    boot2 = con.read_until(r"Calibrated gyro bias at", 150)
    m = RE_BIAS.search(boot2)
    con.read_until(r"Saved configuration", 5)
    con.close()
    if m is None:
        sys.exit("the firmware stored no new point in 150 s (did the tracker move?). Try again.")
    new_t = float(m[1])
    new_b = [float(m[2]), float(m[3]), float(m[4])]
    print(f"  new point: {new_t:.1f} C  bias {new_b}")

    print("[3/3] comparando")
    result = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "port": port,
              "mac": mac[1] if mac else None, "firmware": fw[1],
              "old_points": old, "new_point": [new_t, new_b]}
    if old:
        est = interp(old, new_t)
        err = [n - e for n, e in zip(new_b, est)]
        deg_min = [e * GYRO_DPS_PER_LSB * 60 for e in err]
        result["old_estimate_at_new_t"] = est
        result["error_deg_per_min"] = deg_min
        print(f"  the old calibration estimated {[round(x, 3) for x in est]} at {new_t:.1f} C")
        print(f"  per-axis error: {', '.join(f'{d:+.2f}' for d in deg_min)}  deg/min"
              f"  (maior: {max(abs(d) for d in deg_min):.2f} °/min)")

    log_path = FW_DIR / f"recal_{(mac[1] if mac else 'unknown').replace(':', '')}_{time.strftime('%Y%m%d-%H%M%S')}.txt"
    log_path.write_text(con.log, encoding="utf-8")
    with open(FW_DIR / "calibrations.jsonl", "a", encoding="utf-8") as f:
        f.write(json.dumps(result) + "\n")
    print(f"\nlog: {log_path}")


if __name__ == "__main__":
    main()
