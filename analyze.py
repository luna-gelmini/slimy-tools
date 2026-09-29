"""Analyse sessions recorded by record.py.

    python analyze.py sessions/20260911-003000_night        # full report
    python analyze.py sessions/*chest*                      # several: comparison table

For each tracker (and each position it was mounted in):
  - % of time locked by Stay Aligned, and how often it unlocked
  - drift rate at rest (deg/min), per locked stretch of >= 60 s, measured
    both from the Stay Aligned correction and from the raw quaternion yaw
  - estimated drift accumulated while unlocked (what Stay Aligned does not
    recover): rest rate x unlocked time. Only an estimate: while moving the
    real rate can differ.
  - distribution of the |acceleration| MAD over 2 s windows, locked vs
    unlocked (a basis for calibrating thresholds)
  - battery (%/h, V/h) and temperature
  - resets: the yaw jump each tracker took (~ the error accumulated so far)
"""

import csv
import gzip
import json
import sys
import time
import zlib
from collections import defaultdict
from pathlib import Path

import quatmath as qm

MIN_SEGMENT_S = 60.0
GAP_S = 30.0
ACC_BIN_S = 2.0


def read_samples(path):
    rows = []
    try:
        with gzip.open(path, "rt", newline="", encoding="utf-8") as f:
            for r in csv.DictReader(f):
                rows.append(r)
    except (EOFError, zlib.error, gzip.BadGzipFile):
        pass
    out = []
    for r in rows:
        try:
            out.append({
                "t": float(r["t"]), "key": r["key"], "bp": r["body_part"],
                "q": tuple(float(r[c]) for c in ("qw", "qx", "qy", "qz")) if r["qw"] else None,
                "qa": tuple(float(r[c]) for c in ("aqw", "aqx", "aqy", "aqz")) if r["aqw"] else None,
                "acc": (float(r["lax"]) ** 2 + float(r["lay"]) ** 2 + float(r["laz"]) ** 2) ** 0.5 if r["lax"] else None,
                "temp": float(r["temp"]) if r["temp"] else None,
                "sa": float(r["sa_corr"]) if r["sa_corr"] else None,
                "locked": r["sa_locked"] == "1",
            })
        except (KeyError, ValueError):
            continue
    return out


def read_events(path):
    if not path.exists():
        return []
    evs = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            evs.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return evs


def read_devices(path):
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def locked_segments(rows, reset_times):
    """Split locked samples into continuous stretches, breaking at unlocks,
    connection gaps and resets (which zero sa_corr)."""
    segs, cur = [], []
    ri = 0
    for r in rows:
        while ri < len(reset_times) and reset_times[ri] < r["t"]:
            if cur and cur[-1]["t"] < reset_times[ri]:
                segs.append(cur)
                cur = []
            ri += 1
        if not r["locked"] or r["sa"] is None or r["q"] is None:
            if cur:
                segs.append(cur)
            cur = []
            continue
        if cur and r["t"] - cur[-1]["t"] > GAP_S:
            segs.append(cur)
            cur = []
        cur.append(r)
    if cur:
        segs.append(cur)
    return [s for s in segs if s[-1]["t"] - s[0]["t"] >= MIN_SEGMENT_S]


def segment_rates(seg):
    ts = [r["t"] for r in seg]
    sa_rate = -qm.slope(ts, [r["sa"] for r in seg]) * 60
    yaw, acc = [0.0], 0.0
    for a, b in zip(seg, seg[1:]):
        acc += qm.world_yaw_delta_deg(a["q"], b["q"])
        yaw.append(acc)
    raw_rate = qm.slope(ts, yaw) * 60
    return sa_rate, raw_rate, ts[-1] - ts[0]


QUIET_WIN_S = 120.0
QUIET_MAX_DEG = 8.0
QUIET_STEP_S = 10.0


def quiet_window_rates(rows):
    """Drift from the raw yaw, independent of Stay Aligned.

    Uses 2-minute windows where the tracker never moved more than 8 deg from
    its starting orientation (no real repositioning). Breathing oscillates and
    cancels out; what remains as a yaw trend is drift. Returns deg/min rates.
    """
    rates = []
    n = len(rows)
    next_start_t = -1.0
    last_accepted_end = -1.0
    for i in range(n):
        start = rows[i]
        if start["q"] is None or start["t"] < next_start_t or start["t"] < last_accepted_end:
            continue
        next_start_t = start["t"] + QUIET_STEP_S
        ts, yaw, acc, ok = [start["t"]], [0.0], 0.0, True
        j = i
        while j + 1 < n and rows[j + 1]["t"] - start["t"] <= QUIET_WIN_S:
            a, b = rows[j], rows[j + 1]
            j += 1
            if a["q"] is None or b["q"] is None or b["t"] - a["t"] > GAP_S:
                ok = False
                break
            if qm.angle_deg(start["q"], b["q"]) > QUIET_MAX_DEG:
                ok = False
                break
            acc += qm.world_yaw_delta_deg(a["q"], b["q"])
            ts.append(b["t"])
            yaw.append(acc)
        if ok and ts[-1] - ts[0] >= QUIET_WIN_S * 0.9:
            rates.append(qm.slope(ts, yaw) * 60)
            last_accepted_end = ts[-1]
    return rates


def acc_mad_bins(rows):
    bins = defaultdict(list)
    for r in rows:
        if r["acc"] is not None:
            bins[int(r["t"] // ACC_BIN_S)].append(r)
    locked, moving = [], []
    for rs in bins.values():
        if len(rs) < 3:
            continue
        m = qm.mad([r["acc"] for r in rs])
        (locked if all(r["locked"] for r in rs) else moving).append(m)
    return locked, moving


def reset_jumps(rows_by_key, events):
    out = []
    for e in events:
        if e["kind"] != "reset" or e.get("status") != "FINISHED":
            continue
        t = e["t"]
        jumps = {}
        for key, rows in rows_by_key.items():
            before = [r for r in rows if t - 5 <= r["t"] < t and r["qa"]]
            after = [r for r in rows if t + 1.5 <= r["t"] <= t + 6 and r["qa"]]
            if before and after:
                jumps[key] = qm.world_yaw_delta_deg(before[-1]["qa"], after[0]["qa"])
        out.append((e, jumps))
    return out


def analyze_session(sdir):
    sdir = Path(sdir)
    meta = json.loads((sdir / "meta.json").read_text(encoding="utf-8"))
    rows = read_samples(sdir / "samples.csv.gz")
    events = read_events(sdir / "events.jsonl")
    devices = read_devices(sdir / "devices.csv")
    reset_times = sorted(e["t"] for e in events if e["kind"] == "reset")

    rows_by_key = defaultdict(list)
    for r in rows:
        rows_by_key[r["key"]].append(r)

    results = []
    for key, krows in rows_by_key.items():
        name = meta["trackers"].get(key, {}).get("name", key)
        by_bp = defaultdict(list)
        for r in krows:
            by_bp[r["bp"]].append(r)
        for bp, brows in by_bp.items():
            dur = brows[-1]["t"] - brows[0]["t"]
            locked_t = unlocked_t = 0.0
            unlocks = 0
            for a, b in zip(brows, brows[1:]):
                dt = b["t"] - a["t"]
                if dt > GAP_S:
                    continue
                if a["locked"]:
                    locked_t += dt
                else:
                    unlocked_t += dt
                if a["locked"] and not b["locked"]:
                    unlocks += 1

            sa_total = 0.0
            sa_t = 0.0
            for a, b in zip(brows, brows[1:]):
                if a["sa"] is None or b["sa"] is None or b["t"] - a["t"] > GAP_S:
                    continue
                if any(a["t"] <= rt < b["t"] for rt in reset_times):
                    continue
                sa_total += b["sa"] - a["sa"]
                sa_t += b["t"] - a["t"]
            sa_total_rate = -sa_total / sa_t * 60 if sa_t >= MIN_SEGMENT_S else None

            quiet = quiet_window_rates(brows)

            segs = locked_segments(brows, reset_times)
            rates = [segment_rates(s) for s in segs]
            total_w = sum(r[2] for r in rates)
            sa_rate = sum(r[0] * r[2] for r in rates) / total_w if total_w else None
            raw_rate = sum(r[1] * r[2] for r in rates) / total_w if total_w else None
            seg_sa = [r[0] for r in rates]

            mad_locked, mad_moving = acc_mad_bins(brows)
            temps = [r["temp"] for r in brows if r["temp"] is not None]

            dev = [d for d in devices if d["key"] == key and d["battery_v"]]
            bat = None
            if len(dev) >= 2:
                h = (float(dev[-1]["t"]) - float(dev[0]["t"])) / 3600
                if h >= 10 / 60:
                    dv = (float(dev[-1]["battery_v"]) - float(dev[0]["battery_v"])) / h
                    dp = None
                    if dev[0]["battery_pct"] and dev[-1]["battery_pct"]:
                        dp = (float(dev[-1]["battery_pct"]) - float(dev[0]["battery_pct"])) / h
                    bat = (dv, dp)

            results.append({
                "key": key, "name": name, "bp": bp, "dur": dur,
                "locked_pct": 100 * locked_t / (locked_t + unlocked_t) if locked_t + unlocked_t else None,
                "unlocks": unlocks, "unlocked_min": unlocked_t / 60,
                "sa_rate": sa_rate, "raw_rate": raw_rate, "n_segs": len(segs),
                "sa_total_rate": sa_total_rate,
                "quiet_rate": qm.median(quiet) if quiet else None, "n_quiet": len(quiet),
                "seg_p10": qm.percentile(seg_sa, 10) if seg_sa else None,
                "seg_p90": qm.percentile(seg_sa, 90) if seg_sa else None,
                "est_unrecovered": abs(sa_rate) * unlocked_t / 60 if sa_rate is not None else None,
                "mad_locked": mad_locked, "mad_moving": mad_moving,
                "rows_per_s": len(brows) / dur if dur else None,
                "temp": (min(temps), max(temps)) if temps else None,
                "bat": bat,
            })

    return meta, results, reset_jumps(rows_by_key, events), events


def f(v, spec=".2f", none="—"):
    return none if v is None else format(v, spec)


def report(sdir):
    meta, results, jumps, events = analyze_session(sdir)
    dur_h = ((meta.get("end") or max((r["dur"] for r in results), default=0) + meta["start"]) - meta["start"]) / 3600
    out = [f"# {meta['label']}  ({Path(sdir).name})", ""]
    if meta.get("note"):
        out += [f"Note: {meta['note']}", ""]
    out += [f"Duration: {dur_h * 60:.0f} min", ""]

    out += ["## Drift per tracker", "",
            "`quiet drift` = **main metric**: trend of the raw yaw (before Stay Aligned) over 2-minute windows "
            "with no real repositioning; median across windows. "
            "`SA correction` = how much Stay Aligned corrected per minute (underestimates trackers that unlock often). "
            "`rest drift` = rate measured only in locked stretches of 60 s or more.", "",
            "| position | tracker | quiet drift | locked | unlocks | SA correction | rest drift SA | raw yaw | stretch range (p10-p90) | unrecovered drift (est.) | samples/s | tempC | battery |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(results, key=lambda r: r["bp"]):
        bat = "—" if not r["bat"] else f"{f(r['bat'][1], '+.1f')} %/h ({r['bat'][0]:+.3f} V/h)"
        temp = "—" if not r["temp"] else f"{r['temp'][0]:.1f}–{r['temp'][1]:.1f}"
        out.append(
            f"| {r['bp']} | {r['name']} | **{f(r['quiet_rate'], '+.2f')} °/min** ({r['n_quiet']} jan.) "
            f"| {f(r['locked_pct'], '.0f')}% | {r['unlocks']} "
            f"| {f(r['sa_total_rate'], '+.2f')} °/min | {f(r['sa_rate'], '+.2f')} °/min ({r['n_segs']} trechos) | {f(r['raw_rate'], '+.2f')} °/min "
            f"| {f(r['seg_p10'], '+.2f')} … {f(r['seg_p90'], '+.2f')} "
            f"| {f(r['est_unrecovered'], '.1f')}° em {r['unlocked_min']:.0f} min "
            f"| {f(r['rows_per_s'], '.1f')} | {temp} | {bat} |")

    out += ["", "## |acceleration| MAD over 2 s windows (m/s^2)", "",
            "Basis for calibrating `accel_thresh`: it should sit above almost all rest and below movement.", "",
            "| position | tracker | locked p50 | locked p95 | locked p99 | unlocked p5 | unlocked p50 |",
            "|---|---|---|---|---|---|---|"]
    for r in sorted(results, key=lambda r: r["bp"]):
        L, M = r["mad_locked"], r["mad_moving"]
        out.append(f"| {r['bp']} | {r['name']} | {f(qm.percentile(L, 50) if L else None, '.4f')} "
                   f"| {f(qm.percentile(L, 95) if L else None, '.4f')} | {f(qm.percentile(L, 99) if L else None, '.4f')} "
                   f"| {f(qm.percentile(M, 5) if M else None, '.4f')} | {f(qm.percentile(M, 50) if M else None, '.4f')} |")

    names = {r["key"]: f"{r['bp']}" for r in results}
    out += ["", "## Resets", "",
            "A yaw reset aligns every tracker to the head yaw. The jump only measures accumulated drift if "
            "the pose is the same at both resets (e.g. always standing, straight, facing forward) and you "
            "stay still during the countdown. Sitting or moving, the jump mixes pose with drift.", ""]
    if not jumps:
        out.append("No resets during the session.")
    for e, js in jumps:
        when = time.strftime("%H:%M:%S", time.localtime(e["t"]))
        parts = ", ".join(f"{names.get(k, k)} {v:+.1f} deg" for k, v in sorted(js.items(), key=lambda kv: names.get(kv[0], kv[0])))
        out.append(f"- {when} {e['reset_type']}: {parts or 'no samples around it'}")

    moves = [e for e in events if e["kind"] == "body_part"]
    if moves:
        out += ["", "## Position changes", ""]
        out += [f"- {e['name']}: {e['old']} → {e['new']}" for e in moves]

    text = "\n".join(out) + "\n"
    (Path(sdir) / "report.md").write_text(text, encoding="utf-8")
    return text


def compare(sdirs):
    out = ["| session | position | tracker | quiet drift deg/min | locked | unlocks/min | SA correction deg/min | rest drift SA | raw yaw | samples/s | battery %/h |",
           "|---|---|---|---|---|---|---|---|---|---|---|"]
    for sdir in sdirs:
        meta, results, _, _ = analyze_session(sdir)
        for r in sorted(results, key=lambda r: r["bp"]):
            out.append(f"| {meta['label']} | {r['bp']} | {r['name']} | {f(r['quiet_rate'], '+.2f')} ({r['n_quiet']}) "
                       f"| {f(r['locked_pct'], '.0f')}% "
                       f"| {f(r['unlocks'] / (r['dur'] / 60) if r['dur'] else None, '.1f')} "
                       f"| {f(r['sa_total_rate'], '+.2f')} | {f(r['sa_rate'], '+.2f')} | {f(r['raw_rate'], '+.2f')} | {f(r['rows_per_s'], '.1f')} "
                       f"| {f(r['bat'][1] if r['bat'] else None, '+.1f')} |")
    return "\n".join(out)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    dirs = [p for p in sys.argv[1:] if (Path(p) / "meta.json").exists()]
    if not dirs:
        print(__doc__)
        sys.exit(1)
    print(report(dirs[0]) if len(dirs) == 1 else compare(dirs))
