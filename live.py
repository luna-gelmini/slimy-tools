"""Live monitor: drift rate per tracker, Stay Aligned state, temperature.

    python live.py               # redraws every 2 s, Ctrl+C to exit
    python live.py --seconds 60  # measures for 60 s and prints a final table

How the drift rate is measured
------------------------------
While a tracker is locked (at rest), Stay Aligned assumes any yaw rotation
is drift and undoes it. So the rate at which the accumulated correction
(sa_corr) grows is the tracker's own drift rate: Stay Aligned doubles as a
drift sensor.

The "raw yaw" column measures the same thing straight from the fused
quaternion (twist around the vertical axis), as an independent check.
"""

import argparse
import asyncio
import sys
import time
from collections import deque

import quatmath as qm
from slimevr_client import ConnectionEvent, FeedUpdate, ResetEvent, SlimeVRClient

WINDOW_S = 60.0
ACC_WINDOW_S = 5.0


class TrackerState:
    def __init__(self):
        self.hist = deque()
        self.last_key = None
        self.last = None

    def push(self, t, s):
        self.last = s
        sig = (s.quat, s.lin_acc)
        if sig == self.last_key:
            return
        self.last_key = sig
        self.hist.append((t, s.quat, s.sa_corr, s.sa_locked, s.lin_acc_mag))
        while self.hist and t - self.hist[0][0] > WINDOW_S:
            self.hist.popleft()

    def locked_tail(self):
        """Samples from the most recent locked stretch (no unlock in between)."""
        tail = []
        for h in reversed(self.hist):
            if not h[3]:
                break
            tail.append(h)
        return tail[::-1]

    def sa_drift_deg_min(self):
        tail = self.locked_tail()
        if len(tail) < 5 or tail[-1][0] - tail[0][0] < 10:
            return None
        return -qm.slope([h[0] for h in tail], [h[2] for h in tail]) * 60

    def raw_yaw_deg_min(self):
        tail = self.locked_tail()
        if len(tail) < 5 or tail[-1][0] - tail[0][0] < 10:
            return None
        ts, yaw, acc = [tail[0][0]], [0.0], 0.0
        for a, b in zip(tail, tail[1:]):
            acc += qm.world_yaw_delta_deg(a[1], b[1])
            ts.append(b[0])
            yaw.append(acc)
        return qm.slope(ts, yaw) * 60

    def acc_mad(self, now):
        vals = [h[4] for h in self.hist if now - h[0] <= ACC_WINDOW_S and h[4] is not None]
        return qm.mad(vals) if len(vals) >= 3 else None

    def unique_rate(self, now):
        n = sum(1 for h in self.hist if now - h[0] <= ACC_WINDOW_S)
        return n / ACC_WINDOW_S


def fmt(v, spec, width):
    return f"{'—' if v is None else format(v, spec):>{width}}"


def render(states, now, events):
    lines = [
        f"{'position':<16}{'tracker':<15}{'bat':>11}{'tempC':>7}{'lock':>6}{'SA corr':>10}"
        f"{'drift SA':>10}{'raw yaw':>11}{'|acc| MAD':>11}{'new/s':>10}",
        f"{'':<16}{'':<15}{'':>11}{'':>7}{'':>6}{'':>10}{'°/min':>10}{'°/min':>11}{'m/s²':>11}{'':>10}",
    ]
    for key, st in sorted(states.items(), key=lambda kv: kv[1].last.body_part):
        s = st.last
        bat = "—" if s.battery_v is None else f"{s.battery_pct}% {s.battery_v:.2f}"
        lines.append(
            f"{s.body_part[:15]:<16}{s.name[:14]:<15}{bat:>11}"
            f"{fmt(s.temp, '.1f', 7)}{('yes' if s.sa_locked else 'no'):>6}"
            f"{fmt(s.sa_corr, '.2f', 10)}{fmt(st.sa_drift_deg_min(), '+.2f', 10)}"
            f"{fmt(st.raw_yaw_deg_min(), '+.2f', 11)}{fmt(st.acc_mad(now), '.4f', 11)}"
            f"{st.unique_rate(now):>10.1f}"
        )
    if events:
        lines.append("")
        lines += [f"  {time.strftime('%H:%M:%S', time.localtime(t))}  {msg}" for t, msg in events[-6:]]
    return "\n".join(lines)


async def main(seconds):
    client = SlimeVRClient()
    states, events = {}, []
    t_start = time.time()
    next_draw = t_start + 2

    async for msg in client.messages():
        now = time.time()
        if isinstance(msg, ConnectionEvent):
            events.append((now, ("connected " if msg.connected else "disconnected: ") + msg.detail))
            if not msg.connected:
                print(events[-1][1])
        elif isinstance(msg, ResetEvent):
            parts = ",".join(msg.body_parts) or "all"
            events.append((now, f"reset {msg.reset_type} {msg.status} ({parts})"))
        elif isinstance(msg, FeedUpdate):
            for s in msg.trackers:
                if s.is_imu:
                    states.setdefault(s.key, TrackerState()).push(msg.t, s)

        if seconds is None and now >= next_draw and states:
            next_draw = now + 2
            print("\x1b[2J\x1b[H" + render(states, now, events), flush=True)
        if seconds is not None and now - t_start >= seconds:
            print(render(states, now, events))
            return


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, help="measure for N seconds, then print a final table")
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")
    try:
        asyncio.run(main(args.seconds))
    except KeyboardInterrupt:
        pass
