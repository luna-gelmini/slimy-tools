"""Log the raw IMU stream over Wi-Fi (firmware 0.7.3+i2cbat.4udp and later).

    python tools/raw_udp_log.py --discover
    python tools/raw_udp_log.py --minutes 60 --interval 200
    python tools/raw_udp_log.py --ip 192.168.0.22 --minutes 480 --label night

Sends "RAW ON <ms>" to every tracker that answers "RAW PING" (or just to
--ip), receives the [RAW] lines and writes them to raw/<date>_udp_<label>.csv.
Sends "RAW OFF" on the way out.

Unlike the serial logger this works while the trackers are worn, so the gyro
bias can be measured on the body, overnight included. SlimeVR can stay open:
this uses UDP port 6971, not the server's ports.

Columns: t_pc, ip, sensor, millis, gx, gy, gz, ax, ay, az, tempC, n
(gx..az in raw sensor LSB; on the LSM6DSV at 1000 dps, 1 LSB = 0.035 deg/s)
"""

import argparse
import csv
import socket
import sys
import time
from pathlib import Path

PORT = 6971
DEST = Path(__file__).resolve().parent.parent / "raw"


def open_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(("0.0.0.0", 0))
    s.settimeout(0.5)
    return s


def broadcast_addresses():
    """Directed broadcast for each local IPv4 (assumes /24), plus the global one.

    Windows and most routers drop 255.255.255.255, so the directed address is
    what actually reaches the trackers."""
    addrs = {"255.255.255.255"}
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 53))
        local = probe.getsockname()[0]
        addrs.add(local.rsplit(".", 1)[0] + ".255")
    except OSError:
        pass
    finally:
        probe.close()
    return sorted(addrs)


def discover(s, seconds=3.0):
    for addr in broadcast_addresses():
        s.sendto(b"RAW PING", (addr, PORT))
    found = {}
    end = time.time() + seconds
    while time.time() < end:
        try:
            data, addr = s.recvfrom(256)
        except socket.timeout:
            continue
        if data.startswith(b"OK"):
            found[addr[0]] = data.decode(errors="replace").strip()
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", action="append", default=[], help="tracker address; repeat for several. Default: discover")
    ap.add_argument("--minutes", type=float, default=30)
    ap.add_argument("--interval", type=int, default=200, help="ms between lines")
    ap.add_argument("--label", default="")
    ap.add_argument("--discover", action="store_true", help="only list the trackers that answer")
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")

    s = open_socket()

    targets = args.ip
    if not targets or args.discover:
        print(f"looking for trackers on UDP {PORT}...")
        found = discover(s)
        for ip, reply in sorted(found.items()):
            print(f"  {ip}  {reply}")
        if not found:
            print("  none answered. Is the tracker on the i2cbat-raw firmware and connected?")
        if args.discover:
            return
        targets = sorted(found)
    if not targets:
        sys.exit(1)

    DEST.mkdir(exist_ok=True)
    name = f"{time.strftime('%Y%m%d-%H%M%S')}_udp{('_' + args.label) if args.label else ''}.csv"
    path = DEST / name

    for ip in targets:
        s.sendto(f"RAW ON {args.interval}".encode(), (ip, PORT))
    print(f"streaming from {', '.join(targets)} every {args.interval} ms")
    print(f"writing to {path}  (Ctrl+C to stop)")

    end = time.time() + args.minutes * 60
    n = 0
    last_seen = {}
    try:
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_pc", "ip", "sensor", "millis", "gx", "gy", "gz", "ax", "ay", "az", "tempC", "n"])
            next_flush = time.time() + 10
            while time.time() < end:
                try:
                    data, addr = s.recvfrom(512)
                except socket.timeout:
                    continue
                txt = data.decode("utf-8", "replace").strip()
                if not txt.startswith("[RAW]"):
                    continue
                fields = txt.split()[1:]
                if len(fields) != 10:
                    continue
                w.writerow([round(time.time(), 3), addr[0], *fields])
                n += 1
                last_seen[addr[0]] = time.time()
                if time.time() >= next_flush:
                    next_flush = time.time() + 10
                    f.flush()
                    quiet = [ip for ip in targets if time.time() - last_seen.get(ip, 0) > 30]
                    status = f"  {n} rows  temp {fields[8]}C"
                    if quiet:
                        status += f"  [silent: {', '.join(quiet)}]"
                    print(status, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        for ip in targets:
            s.sendto(b"RAW OFF", (ip, PORT))
        s.close()
    print(f"\n{n} rows saved to {path}")


if __name__ == "__main__":
    main()
