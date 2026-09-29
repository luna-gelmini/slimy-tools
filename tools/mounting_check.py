"""Check leg tracker mounting by comparing the left and right sides.

    python tools/mounting_check.py                 # guided: standing, then lying
    python tools/mounting_check.py --pose lying    # capture a single pose
    python tools/mounting_check.py --report        # re-print the last captures

Mounting calibration infers how a tracker is strapped on from the direction the
limb moves, assuming that movement happens in the body's sagittal plane. Doing
it with the legs splayed rotates the stored mounting by roughly the splay
angle. That error is invisible while standing, because a twist around a
vertical limb does not change where the limb points, and only shows up once the
limb leaves vertical: one leg reads lower than the other when sitting or lying.

This compares the two legs in poses where they are physically parallel, so
anything above a few degrees is mounting, not posture:

    standing   upright, feet together, arms relaxed
    lying      on your back, legs straight, ankles touching

The measured quantity is the angle between the two limbs' long axes (tracker
local +Y from rotation_reference_adjusted), which does not depend on which way
the body is facing.
"""

import argparse
import asyncio
import json
import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import quatmath as qm  # noqa: E402
from slimevr_client import ConnectionEvent, FeedUpdate, SlimeVRClient  # noqa: E402

DEST = Path(__file__).resolve().parent.parent / "raw" / "mounting"
PAIRS = [
    ("LEFT_UPPER_LEG", "RIGHT_UPPER_LEG", "thighs"),
    ("LEFT_LOWER_LEG", "RIGHT_LOWER_LEG", "shins"),
]
GOOD, SUSPECT = 5.0, 10.0


def limb_axis(quat):
    r = qm.mul(qm.mul(quat, (0.0, 0.0, 1.0, 0.0)), qm.conj(quat))
    return r[1:]


def normalise(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return [c / n for c in v]


def angle_between(a, b):
    dot = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
    return math.degrees(math.acos(dot))


async def capture(seconds):
    """Average each tracker's limb axis over `seconds` of a held pose."""
    client = SlimeVRClient(interval_ms=100)
    axes, names = {}, {}
    end = None
    async for msg in client.messages():
        if isinstance(msg, ConnectionEvent):
            if not msg.connected:
                sys.exit(f"SlimeVR server not reachable: {msg.detail}")
            continue
        if not isinstance(msg, FeedUpdate):
            continue
        if end is None:
            end = time.time() + seconds
        for s in msg.trackers:
            if not s.is_imu or s.quat_adj is None:
                continue
            axes.setdefault(s.body_part, []).append(limb_axis(s.quat_adj))
            names[s.body_part] = s.name
        if time.time() >= end:
            break
    return (
        {bp: normalise([statistics.mean(v[i] for v in vs) for i in range(3)]) for bp, vs in axes.items()},
        names,
        {bp: len(vs) for bp, vs in axes.items()},
    )


def verdict(angle):
    if angle <= GOOD:
        return "ok"
    if angle <= SUSPECT:
        return "borderline"
    return "MOUNTING ERROR"


def report(poses):
    print(f"\n{'pose':10}{'pair':8}{'angle':>8}   verdict")
    for pose, data in poses.items():
        axes = data["axes"]
        for left, right, label in PAIRS:
            if left not in axes or right not in axes:
                print(f"{pose:10}{label:8}{'—':>8}   missing tracker")
                continue
            a = angle_between(axes[left], axes[right])
            print(f"{pose:10}{label:8}{a:8.1f}   {verdict(a)}")
    print(
        "\nIn both poses the legs are parallel, so the angle should be a few degrees.\n"
        "A large angle only while lying, with standing fine, is the signature of a\n"
        "mounting calibration done with the legs apart. Redo it with the feet\n"
        "parallel and close together."
    )


def load_saved():
    poses = {}
    for pose in ("standing", "lying"):
        f = DEST / f"{pose}.json"
        if f.exists():
            poses[pose] = json.loads(f.read_text(encoding="utf-8"))
    return poses


def run_pose(pose, seconds):
    print(f"\n{pose}: hold the pose, then press Enter to record {seconds:.0f} s.")
    if pose == "standing":
        print("  upright, feet together, arms relaxed.")
    else:
        print("  on your back, legs straight, ankles touching.")
    input("  ready? ")
    print("  recording, hold still...", flush=True)
    axes, names, counts = asyncio.run(capture(seconds))
    if not axes:
        sys.exit("no IMU trackers reported anything")
    DEST.mkdir(parents=True, exist_ok=True)
    data = {"pose": pose, "t": time.time(), "axes": axes, "names": names, "samples": counts}
    (DEST / f"{pose}.json").write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"  captured {min(counts.values())}-{max(counts.values())} samples per tracker")
    return data


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", choices=["standing", "lying"], help="capture a single pose")
    ap.add_argument("--seconds", type=float, default=30)
    ap.add_argument("--report", action="store_true", help="only re-print the last captures")
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")

    if args.report:
        poses = load_saved()
        if not poses:
            sys.exit(f"nothing saved in {DEST}")
        report(poses)
        return

    if args.pose:
        poses = load_saved()
        poses[args.pose] = run_pose(args.pose, args.seconds)
    else:
        poses = {p: run_pose(p, args.seconds) for p in ("standing", "lying")}
    report(poses)


if __name__ == "__main__":
    main()
