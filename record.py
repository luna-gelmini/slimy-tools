"""Session recorder: everything SlimeVR sends, plus events, until Ctrl+C.

    python record.py --label night
    python record.py --label chest-sternum --note "tracker C3A86, strap centred" --minutes 20

Creates sessions/<timestamp>_<label>/ with:
    meta.json       label, note, start/end, trackers seen
    samples.csv.gz  IMU samples (only when something changes, or every 5 s)
    devices.csv     battery / signal / temperature every 30 s
    events.jsonl    connections, resets (tap resets included), Stay Aligned
                    lock/unlock, tracker position changes

The .gz is flushed every 10 s, so a power cut in the middle of the night
costs at most the last few seconds.

To stop without Ctrl+C (e.g. when running in the background), create a file
named STOP inside the session folder.
"""

import argparse
import asyncio
import csv
import gzip
import json
import sys
import time
from pathlib import Path

from slimevr_client import ConnectionEvent, FeedUpdate, ResetEvent, SlimeVRClient

SESSIONS_DIR = Path(__file__).parent / "sessions"
FORCE_ROW_S = 5.0
DEVICE_ROW_S = 30.0
FLUSH_S = 10.0
STATUS_S = 60.0

SAMPLE_COLS = [
    "t", "key", "body_part",
    "qw", "qx", "qy", "qz",
    "aqw", "aqx", "aqy", "aqz",
    "lax", "lay", "laz",
    "temp", "tps",
    "sa_corr", "sa_locked", "sa_locked_err", "sa_center_err", "sa_neighbor_err",
]
DEVICE_COLS = ["t", "key", "name", "body_part", "battery_v", "battery_pct", "rssi", "temp"]


def _r(v, nd=6):
    return "" if v is None else round(v, nd)


class Recorder:
    def __init__(self, out_dir, label, note):
        self.dir = out_dir
        self.dir.mkdir(parents=True)
        self.meta = {"label": label, "note": note, "start": time.time(), "end": None, "trackers": {}}
        self.samples_f = gzip.open(self.dir / "samples.csv.gz", "wt", newline="", encoding="utf-8")
        self.samples = csv.writer(self.samples_f)
        self.samples.writerow(SAMPLE_COLS)
        self.devices_f = open(self.dir / "devices.csv", "w", newline="", encoding="utf-8")
        self.devices = csv.writer(self.devices_f)
        self.devices.writerow(DEVICE_COLS)
        self.events_f = open(self.dir / "events.jsonl", "w", encoding="utf-8")

        self.last_sig = {}
        self.last_row_t = {}
        self.last_dev_t = {}
        self.locked = {}
        self.body_part = {}
        self.rows = 0
        self.last_flush = time.time()
        self._write_meta()

    def _write_meta(self):
        (self.dir / "meta.json").write_text(json.dumps(self.meta, indent=2, ensure_ascii=False), encoding="utf-8")

    def event(self, t, kind, **data):
        self.events_f.write(json.dumps({"t": round(t, 3), "kind": kind, **data}, ensure_ascii=False) + "\n")

    def on_update(self, u):
        for s in u.trackers:
            if not s.is_imu:
                continue
            t = u.t

            if s.key not in self.meta["trackers"]:
                self.meta["trackers"][s.key] = {"name": s.name, "imu": s.imu, "body_part": s.body_part}
                self._write_meta()
            if self.body_part.get(s.key) not in (None, s.body_part):
                self.event(t, "body_part", key=s.key, name=s.name, old=self.body_part[s.key], new=s.body_part)
                self.meta["trackers"][s.key]["body_part"] = s.body_part
                self._write_meta()
            self.body_part[s.key] = s.body_part

            if s.sa_locked is not None and self.locked.get(s.key) != s.sa_locked:
                if s.key in self.locked:
                    self.event(t, "lock" if s.sa_locked else "unlock",
                               key=s.key, body_part=s.body_part, sa_corr=_r(s.sa_corr, 3))
                self.locked[s.key] = s.sa_locked

            sig = (s.quat, s.lin_acc, s.sa_locked)
            if sig != self.last_sig.get(s.key) or t - self.last_row_t.get(s.key, 0) >= FORCE_ROW_S:
                self.last_sig[s.key] = sig
                self.last_row_t[s.key] = t
                q = s.quat or (None,) * 4
                a = s.quat_adj or (None,) * 4
                la = s.lin_acc or (None,) * 3
                self.samples.writerow([
                    round(t, 3), s.key, s.body_part,
                    *(_r(v) for v in q), *(_r(v) for v in a), *(_r(v, 4) for v in la),
                    _r(s.temp, 2), s.tps if s.tps is not None else "",
                    _r(s.sa_corr, 3), int(bool(s.sa_locked)), _r(s.sa_locked_err, 3),
                    _r(s.sa_center_err, 3), _r(s.sa_neighbor_err, 3),
                ])
                self.rows += 1

            if t - self.last_dev_t.get(s.key, 0) >= DEVICE_ROW_S:
                self.last_dev_t[s.key] = t
                self.devices.writerow([round(t, 3), s.key, s.name, s.body_part,
                                       _r(s.battery_v, 3), s.battery_pct if s.battery_pct is not None else "",
                                       s.rssi if s.rssi is not None else "", _r(s.temp, 2)])

        if u.t - self.last_flush >= FLUSH_S:
            self.flush()

    def flush(self):
        self.samples_f.flush()
        self.devices_f.flush()
        self.events_f.flush()
        self.last_flush = time.time()
        self.meta["end"] = self.last_flush
        self._write_meta()

    def close(self):
        self.meta["end"] = time.time()
        self._write_meta()
        self.samples_f.close()
        self.devices_f.close()
        self.events_f.close()


async def main(label, note, minutes):
    safe = "".join(c if c.isalnum() or c in "-_" else "-" for c in label)
    out = SESSIONS_DIR / f"{time.strftime('%Y%m%d-%H%M%S')}_{safe}"
    rec = Recorder(out, label, note)
    print(f"recording to {out}  (Ctrl+C to stop)", flush=True)
    t0 = time.time()
    next_status = t0 + STATUS_S
    client = SlimeVRClient()
    try:
        async for msg in client.messages():
            now = time.time()
            if isinstance(msg, FeedUpdate):
                rec.on_update(msg)
            elif isinstance(msg, ResetEvent):
                rec.event(msg.t, "reset", reset_type=msg.reset_type, status=msg.status,
                          body_parts=msg.body_parts, progress=msg.progress, duration=msg.duration)
                print(f"{time.strftime('%H:%M:%S')}  reset {msg.reset_type} {msg.status}", flush=True)
            elif isinstance(msg, ConnectionEvent):
                rec.event(msg.t, "connected" if msg.connected else "disconnected", detail=msg.detail)
                print(f"{time.strftime('%H:%M:%S')}  {'connected' if msg.connected else 'disconnected: ' + msg.detail}",
                      flush=True)

            if now >= next_status:
                next_status = now + STATUS_S
                locked = sum(1 for v in rec.locked.values() if v)
                print(f"{time.strftime('%H:%M:%S')}  {(now - t0) / 60:5.1f} min  {rec.rows} rows  "
                      f"locked {locked}/{len(rec.locked)}", flush=True)
            if minutes is not None and now - t0 >= minutes * 60:
                break
            if (out / "STOP").exists():
                print("STOP file found, finishing", flush=True)
                break
    finally:
        rec.close()
        print(f"session saved: {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", required=True, help="short session name (e.g. night, chest-sternum)")
    ap.add_argument("--note", default="", help="free-form notes (position, tracker, etc.)")
    ap.add_argument("--minutes", type=float, help="stop automatically after N minutes")
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")
    try:
        asyncio.run(main(args.label, args.note, args.minutes))
    except KeyboardInterrupt:
        pass
