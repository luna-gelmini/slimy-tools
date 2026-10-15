"""Unattended maintenance watch for SlimeVR trackers, meant to run on a Pi.

    python3 tools/pi_watch.py --server ws://192.168.0.9:21110 \
        --notify ntfy --ntfy-topic my-topic

Four things, all passive. Nothing is ever changed on the trackers; it only
watches and tells you when something needs doing.

1. Drift watch
   Measures quiet drift the same way analyze.py does: the trend of the raw
   fused yaw over 2-minute windows where the tracker barely moved. A tracker
   with a real bias puts nearly every window on the same side of zero, so the
   alert needs both a large median and a consistent sign. Calibration dies when
   a tracker changes position or the room temperature moves, and this catches
   it before you feel it in game.

2. Mounting watch
   The left/right shin check needs you lying down with the legs parallel, so
   instead of asking, it waits until it sees that pose happen on its own
   (both shins near horizontal and still) and measures then.

3. Battery health log
   Appends voltage, percentage and temperature to a CSV, and reports how long
   the trackers sat above the "full" line, which is what actually wears a
   lithium cell.

4. Seasonal recalibration reminder
   Records each tracker's usual working temperature and warns when it drifts
   away from the baseline, because a calibration taken at 34 C stops matching
   when the room is 6 degrees colder.

State lives in <state-dir> so restarts don't lose history.
"""

import argparse
import asyncio
import csv
import json
import math
import statistics
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import notify  # noqa: E402
import quatmath as qm  # noqa: E402
from slimevr_client import ConnectionEvent, FeedUpdate, SlimeVRClient  # noqa: E402

QUIET_WIN_S = 120.0
QUIET_MAX_DEG = 8.0
LYING_MIN_DEG = 60.0
STILL_MAX_DEG = 2.0
BATTERY_FULL_V = 4.10


def limb_axis(quat):
    r = qm.mul(qm.mul(quat, (0.0, 0.0, 1.0, 0.0)), qm.conj(quat))
    return r[1:]


def angle_between(a, b):
    dot = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
    return math.degrees(math.acos(dot))


class TrackerState:
    def __init__(self):
        self.window = []
        self.drifts = deque(maxlen=60)
        self.temps = deque(maxlen=4000)
        self.last_quat = None
        self.last_t = 0.0

    def feed(self, t, quat):
        """Collect 2-minute quiet windows and turn each into a drift rate."""
        if quat is None:
            return None
        if self.window and (t - self.window[0][0] > QUIET_WIN_S
                            or qm.angle_deg(self.window[0][1], quat) > QUIET_MAX_DEG):
            done = self.window
            self.window = [(t, quat)]
            if done[-1][0] - done[0][0] >= QUIET_WIN_S * 0.9:
                ts, yaw, acc = [done[0][0]], [0.0], 0.0
                for a, b in zip(done, done[1:]):
                    acc += qm.world_yaw_delta_deg(a[1], b[1])
                    ts.append(b[0])
                    yaw.append(acc)
                rate = qm.slope(ts, yaw) * 60
                self.drifts.append(rate)
                return rate
            return None
        if not self.window:
            self.window = [(t, quat)]
        elif t - self.window[-1][0] >= 1.0:
            self.window.append((t, quat))
        return None

    def drift_estimate(self, minimum):
        if len(self.drifts) < minimum:
            return None, 0.0
        vals = list(self.drifts)
        positive = sum(1 for v in vals if v > 0) / len(vals)
        return statistics.median(vals), max(positive, 1 - positive)


class Watch:
    def __init__(self, args):
        self.args = args
        self.notify = notify.sender(args)
        self.state_dir = Path(args.state_dir).expanduser()
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.state_file = self.state_dir / "pi_watch.json"
        self.battery_csv = self.state_dir / "battery.csv"
        self.trackers = {}
        self.latest = {}
        self.mounting = deque(maxlen=20)
        self.last_battery_row = 0.0
        self.last_summary_day = None
        self.full_seconds = {}
        self.state = self.load()

    def load(self):
        if self.state_file.exists():
            try:
                return json.loads(self.state_file.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                pass
        return {"baseline_temp": {}, "alerted": {}}

    def save(self):
        self.state_file.write_text(json.dumps(self.state, indent=2), encoding="utf-8")

    def alert_once(self, key, hours, title, message, priority="high"):
        """Avoid repeating the same warning more often than every `hours`."""
        now = time.time()
        if now - self.state["alerted"].get(key, 0) < hours * 3600:
            return
        self.state["alerted"][key] = now
        self.save()
        self.notify(title, message, priority)

    def check_drift(self, name, st, body_part):
        median, agreement = st.drift_estimate(self.args.drift_windows)
        if median is None:
            return
        if abs(median) >= self.args.drift_warn and agreement >= 0.8:
            self.alert_once(
                f"drift:{name}", self.args.alert_cooldown,
                "SlimeVR: tracker drifting",
                f"{name} ({body_part}) is drifting {median:+.2f} deg/min, "
                f"{agreement * 100:.0f}% of windows on the same side over {len(st.drifts)} windows.\n"
                f"Redo the calibration (DELCAL) at its working temperature.",
            )

    def check_mounting(self, samples):
        """When both shins are horizontal and still, they should be parallel."""
        left = samples.get("LEFT_LOWER_LEG")
        right = samples.get("RIGHT_LOWER_LEG")
        if not left or not right or left.quat_adj is None or right.quat_adj is None:
            return
        axes = [limb_axis(left.quat_adj), limb_axis(right.quat_adj)]
        if any(abs(math.degrees(math.asin(max(-1.0, min(1.0, a[1]))))) > 90 - LYING_MIN_DEG for a in axes):
            return
        if not (left.sa_locked and right.sa_locked):
            return
        self.mounting.append(angle_between(*axes))
        if len(self.mounting) < 10:
            return
        median = statistics.median(self.mounting)
        if median >= self.args.mounting_warn:
            self.alert_once(
                "mounting", self.args.alert_cooldown,
                "SlimeVR: mounting looks off",
                f"Lying down, the shins read {median:.1f} deg apart; they are physically parallel, "
                f"so anything above {self.args.mounting_warn:.0f} deg is mounting.\n"
                f"Redo the mounting calibration with the feet parallel and close together.",
            )
            self.mounting.clear()

    def log_battery(self, samples):
        now = time.time()
        if now - self.last_battery_row < self.args.battery_every:
            return
        dt = now - self.last_battery_row if self.last_battery_row else 0
        self.last_battery_row = now
        new = not self.battery_csv.exists()
        with open(self.battery_csv, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["t", "name", "body_part", "volts", "pct", "tempC"])
            for s in samples.values():
                if s.battery_v:
                    w.writerow([round(now, 1), s.name, s.body_part, round(s.battery_v, 3), s.battery_pct,
                                round(s.temp, 2) if s.temp else ""])
                    if dt and s.battery_v >= BATTERY_FULL_V:
                        self.full_seconds[s.name] = self.full_seconds.get(s.name, 0) + dt

    def check_temperature(self, name, st):
        if len(st.temps) < 600:
            return
        current = statistics.median(list(st.temps)[-600:])
        baseline = self.state["baseline_temp"].get(name)
        if baseline is None:
            self.state["baseline_temp"][name] = round(current, 2)
            self.save()
            return
        if abs(current - baseline) >= self.args.temp_warn:
            self.alert_once(
                f"temp:{name}", 24 * 7,
                "SlimeVR: working temperature moved",
                f"{name} now runs at {current:.1f} C, was {baseline:.1f} C when calibrated.\n"
                f"Gyro bias follows temperature, so it is worth redoing the calibration at the new one.",
            )
            self.state["baseline_temp"][name] = round(current, 2)
            self.save()

    def daily_summary(self):
        today = time.strftime("%Y-%m-%d")
        if self.last_summary_day == today or int(time.strftime("%H")) != self.args.summary_hour:
            return
        self.last_summary_day = today
        linhas = []
        for name, st in sorted(self.trackers.items()):
            median, agreement = st.drift_estimate(3)
            temp = statistics.median(st.temps) if st.temps else float("nan")
            full_h = self.full_seconds.get(name, 0) / 3600
            drift = f"{median:+.2f} deg/min ({agreement * 100:.0f}% same sign)" if median is not None else "not enough data"
            linhas.append(f"{name}: drift {drift}, {temp:.1f} C, {full_h:.1f} h above {BATTERY_FULL_V} V")
        mounting = f"{statistics.median(self.mounting):.1f} deg" if len(self.mounting) >= 5 else "not measured"
        self.notify(
            "SlimeVR: daily report",
            "\n".join(linhas) + f"\nShin asymmetry while lying: {mounting}",
            "low",
        )
        self.full_seconds.clear()


async def main(args):
    watch = Watch(args)
    client = SlimeVRClient(url=args.server, interval_ms=500)
    notify.log(f"watching {args.server}, state in {watch.state_dir}")

    async for msg in client.messages():
        if isinstance(msg, ConnectionEvent):
            notify.log(("connected to " if msg.connected else "disconnected: ") + msg.detail)
            continue
        if not isinstance(msg, FeedUpdate):
            continue

        samples = {}
        for s in msg.trackers:
            if not s.is_imu:
                continue
            samples[s.body_part] = s
            watch.latest[s.name] = s
            st = watch.trackers.setdefault(s.name, TrackerState())
            if s.temp:
                st.temps.append(s.temp)
            if st.feed(msg.t, s.quat) is not None:
                watch.check_drift(s.name, st, s.body_part)
                watch.check_temperature(s.name, st)

        watch.check_mounting(samples)
        watch.log_battery({s.name: s for s in samples.values()})
        watch.daily_summary()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", default="ws://192.168.0.9:21110")
    ap.add_argument("--state-dir", default="~/slimy/state")
    ap.add_argument("--drift-warn", type=float, default=0.8, help="deg/min that triggers a drift alert")
    ap.add_argument("--drift-windows", type=int, default=10, help="quiet windows needed before judging")
    ap.add_argument("--mounting-warn", type=float, default=8.0, help="degrees between shins that triggers an alert")
    ap.add_argument("--temp-warn", type=float, default=3.0, help="degrees of working-temperature change to warn about")
    ap.add_argument("--battery-every", type=float, default=300, help="seconds between battery log rows")
    ap.add_argument("--summary-hour", type=int, default=9, help="hour of day for the daily report")
    ap.add_argument("--alert-cooldown", type=float, default=12, help="hours before repeating the same alert")
    notify.add_arguments(ap)
    args = ap.parse_args()
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass
