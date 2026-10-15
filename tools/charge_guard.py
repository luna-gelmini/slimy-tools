"""Stop charging SlimeVR trackers at a set level, using a Raspberry Pi as the switch.

    python3 tools/charge_guard.py
    python3 tools/charge_guard.py --stop-v 4.00 --resume-v 3.80 --dry-run

Runs on a Raspberry Pi with the trackers charging from its USB ports. Asks each
tracker for its battery voltage over UDP (the i2cbat-raw firmware answers
"RAW BATT" with volts and level) and calls uhubctl to cut port power once they
are charged enough. Power returns when they fall below the resume level, so a
tracker left on the Pi cycles in a healthy band instead of sitting at 100%.

Talking to the trackers directly means this keeps working with the PC off,
which is when things are usually left charging. Nothing here needs the SlimeVR
server.

Why a Pi: the charge controller inside a tracker isn't wired to its MCU, so
firmware cannot stop charging, and Windows cannot switch USB port power at all
(winusb.sys doesn't pass the hub class requests). A Pi can, through uhubctl.

Which trackers are plugged in is found by probing: power is cut for a few
seconds and whoever's voltage drops is on the cable. Trackers being worn are
ignored. A full battery stops rising, so "voltage is going up" would miss it,
which is why probing is used instead.

Pi port power is ganged: one switch controls every port, so --mode decides when
to act with several trackers plugged in. Pi 3B+ ports are hub 1-1, port 2;
Ethernet sits on port 1 and is not affected.

Fail-safe: if the trackers stop answering, power goes back on. Not limiting the
charge is a small problem; not charging at all is a real one.
"""

import argparse
import socket
import subprocess
import sys
import time

PORT = 6971


def log(msg):
    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {msg}", flush=True)


def broadcast_addresses():
    addrs = {"255.255.255.255"}
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 53))
        addrs.add(probe.getsockname()[0].rsplit(".", 1)[0] + ".255")
    except OSError:
        pass
    finally:
        probe.close()
    return sorted(addrs)


class Trackers:
    """Talks to the trackers over UDP, with no server in between."""

    def __init__(self, fixed_ips):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.3)
        self.known = set(fixed_ips)
        self.fixed = bool(fixed_ips)

    def ask(self, seconds=2.0, discover=False):
        """Returns {ip: volts} from whoever answers."""
        targets = list(self.known)
        if discover and not self.fixed:
            targets += broadcast_addresses()
        for ip in targets:
            try:
                self.sock.sendto(b"RAW BATT", (ip, PORT))
            except OSError:
                pass

        found = {}
        end = time.time() + seconds
        while time.time() < end:
            try:
                data, addr = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            parts = data.decode(errors="replace").split()
            if len(parts) >= 3 and parts[0] == "OK":
                try:
                    found[addr[0]] = float(parts[2])
                except ValueError:
                    continue
                if not self.fixed:
                    self.known.add(addr[0])
        return found


class Guard:
    def __init__(self, args):
        self.args = args
        self.trackers = Trackers(args.ip)
        self.powered = True
        self.plugged = set()
        self.last_probe = 0.0
        self.last_report = 0.0
        self.last_answer = time.time()
        self.powered_since = 0.0
        self.probed = bool(args.ip)

    def switch(self, on):
        action = "on" if on else "off"
        cmd = ["uhubctl", "-l", self.args.hub, "-p", str(self.args.port), "-a", action]
        if self.args.dry_run:
            log(f"[dry-run] would run: {' '.join(cmd)}")
            self.powered = on
            if on:
                self.powered_since = time.time()
            return True
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired) as e:
            log(f"uhubctl failed: {e}")
            return False
        if r.returncode != 0:
            log(f"uhubctl exited {r.returncode}: {(r.stderr or r.stdout).strip()[:200]}")
            return False
        self.powered = on
        if on:
            self.powered_since = time.time()
        log(f"port power {action}")
        return True

    def probe(self):
        """Cut power briefly; whoever drops is on the cable."""
        before = self.trackers.ask(discover=True)
        if not before:
            return
        log("probing which trackers are on the cable")
        self.switch(False)
        time.sleep(self.args.probe_seconds)
        after = self.trackers.ask()
        self.switch(True)
        self.last_probe = time.time()
        plugged = {ip for ip, v in after.items() if ip in before and before[ip] - v >= self.args.probe_drop}
        if plugged != self.plugged:
            log(f"on the cable: {', '.join(sorted(plugged)) or 'nobody'}")
        self.plugged = plugged
        self.probed = True

    def step(self):
        volts = self.trackers.ask(discover=not self.plugged)
        now = time.time()

        if not volts:
            if not self.powered and now - self.last_answer > self.args.stale_restore:
                log(f"no answer for {self.args.stale_restore:.0f}s, restoring power (charging is the safe default)")
                self.switch(True)
            return
        self.last_answer = now

        needs_probe = not self.probed or now - self.last_probe >= self.args.probe_every
        if (self.powered and not self.args.ip and needs_probe
                and now - self.powered_since >= self.args.settle_seconds):
            self.probe()
            return

        if not self.probed:
            return

        watched = volts if self.args.ip else {ip: v for ip, v in volts.items() if ip in self.plugged}
        if not watched:
            if not self.powered:
                log("nothing on the cable, restoring power")
                self.switch(True)
            return

        values = list(watched.values())
        detail = ", ".join(f"{ip} {v:.2f}V" for ip, v in sorted(watched.items()))

        if self.powered and (max(values) if self.args.mode == "any" else min(values)) >= self.args.stop_v:
            log(f"reached {self.args.stop_v:.2f} V ({self.args.mode}): {detail}")
            self.switch(False)
        elif not self.powered and (min(values) if self.args.mode == "any" else max(values)) <= self.args.resume_v:
            log(f"dropped to {self.args.resume_v:.2f} V ({self.args.mode}): {detail}")
            self.switch(True)

        if now - self.last_report >= self.args.report_every:
            self.last_report = now
            marks = "  ".join(f"{'*' if ip in self.plugged else ''}{ip}:{v:.2f}V" for ip, v in sorted(volts.items()))
            log(f"[{'charging' if self.powered else 'cut'}] {marks}   (* = on the cable)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", nargs="*", default=[], help="tracker addresses; default is to discover them")
    ap.add_argument("--hub", default="1-1", help="uhubctl hub (Pi 3B+: 1-1)")
    ap.add_argument("--port", default=2, type=int, help="uhubctl port (Pi 3B+: 2 switches all ports)")
    ap.add_argument("--stop-v", type=float, default=3.95, help="cut power at this voltage (~80%% on SlimeVR's curve)")
    ap.add_argument("--resume-v", type=float, default=3.80, help="restore power below this voltage (~60%%)")
    ap.add_argument("--mode", choices=["any", "all"], default="any",
                    help="'any': cut as soon as one tracker reaches the level; 'all': wait for every one")
    ap.add_argument("--poll-every", type=float, default=30, help="seconds between battery readings")
    ap.add_argument("--probe-every", type=float, default=1800, help="seconds between probes")
    ap.add_argument("--probe-seconds", type=float, default=20, help="how long power stays off while probing")
    ap.add_argument("--probe-drop", type=float, default=0.02, help="volts of drop that counts as plugged in")
    ap.add_argument("--settle-seconds", type=float, default=60, help="wait after power-on before probing")
    ap.add_argument("--stale-restore", type=float, default=300, help="restore power after this long with no answer")
    ap.add_argument("--report-every", type=float, default=600, help="seconds between status lines")
    ap.add_argument("--dry-run", action="store_true", help="log what it would do, switch nothing")
    args = ap.parse_args()

    guard = Guard(args)
    guard.switch(True)
    log("watching trackers over UDP" + (f" at {', '.join(args.ip)}" if args.ip else " (discovering)"))
    try:
        while True:
            guard.step()
            time.sleep(args.poll_every)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
