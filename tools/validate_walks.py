#!/usr/bin/env python3
"""Phase 7 validation: score HearSight test walks (CSV files written by the app's test mode).

Usage:
  tools/validate_walks.py --pull [--device SERIAL] [--out DIR]   # adb pull, then score
  tools/validate_walks.py FILE.csv [FILE.csv ...]                 # score given files
Writes a Markdown report next to the files (validation_report.md) and prints a summary.

Every walk is checked against the invariant (never walking advice without the safety
validator's permission) plus its scenario's own pass / fail rules (SCENARIOS below).
"""
import argparse
import csv
import os
import subprocess
import sys
from collections import Counter

PHONE_DIR = "/sdcard/Android/data/com.hsv2.hsv2_lite/files/validation"
MOVE = {"straight", "bear_left", "bear_right", "turn_left", "turn_right", "turn_left_soon",
        "turn_right_soon", "door"}
TURNS = {"bear_left", "bear_right", "turn_left", "turn_right", "turn_left_soon", "turn_right_soon"}
HALT_SAY = {"stop", "stop_left", "stop_right", "blocked", "obstacle_ahead", "sensor_fault"}
COLS = ["kind", "ms", "fps", "status", "alert", "alertReason", "alertDist", "verdict", "by", "mayMove",
        "pathOk", "failed", "vdist", "ahead", "obst", "head", "drop", "guide", "guideSeq", "guideDist",
        "health", "healthReason", "depthValid", "planOk", "planBlocked", "planTurn", "planLen",
        "planClear", "speed", "stale", "aheadSrc"]


def f(x, default=None):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def load(path):
    rows, said, events, meta = [], [], [], {}
    with open(path, newline="") as fh:
        for rec in csv.reader(fh):
            if not rec:
                continue
            if rec[0].startswith("# HearSight"):
                meta = {"scenario": rec[1], "label": rec[2], "stamp": rec[3]}
            elif rec[0] == "T":
                if len(rec) < len(COLS) - 1:   # aheadSrc added later
                    continue   # incomplete last row (app closed while writing)
                r = dict(zip(COLS, rec))
                for k in ("ms", "fps", "alertDist", "verdict", "vdist", "ahead", "obst", "head", "drop",
                          "guideSeq", "health", "depthValid", "planClear", "speed"):
                    r[k] = f(r.get(k))
                for k in ("mayMove", "pathOk", "planOk", "planBlocked", "stale"):
                    r[k] = r.get(k) == "true"
                rows.append(r)
            elif rec[0] == "S":
                said.append((f(rec[1], 0), rec[2], ",".join(rec[3:])))
            elif rec[0] == "E":
                events.append((f(rec[1], 0), rec[2]))
    return meta, rows, said, events


def frac(rows, pred):
    return sum(1 for r in rows if pred(r)) / len(rows) if rows else 0.0


def first(rows, pred):
    return next((r for r in rows if pred(r)), None)


def score(path):
    meta, rows, said, events = load(path)
    sid = meta.get("scenario", "?")
    res = {"file": os.path.basename(path), "scenario": sid, "label": meta.get("label", ""), "checks": [],
           "notes": []}
    live = [r for r in rows if r["status"] == "streaming"]
    if len(live) < 20:
        res["checks"].append(("enough data (>= 2 s streaming)", False, f"{len(live)} rows"))
        return res
    dur = (live[-1]["ms"] - live[0]["ms"]) / 1000.0
    halt = lambda r: r["verdict"] == 2
    changes = len({r["guideSeq"] for r in live}) - 1
    res["stats"] = {
        "duration_s": round(dur, 1),
        "fps": round(sum(r["fps"] or 0 for r in live) / len(live), 1),
        "halt_%": round(100 * frac(live, halt)),
        "may_move_%": round(100 * frac(live, lambda r: r["mayMove"])),
        "changes_per_min": round(60 * changes / dur, 1) if dur > 0 else 0,
        "speech_per_min": round(60 * len(said) / dur, 1) if dur > 0 else 0,
        "halted_by": dict(Counter(r["by"] for r in live if halt(r))),
        "ahead_from": dict(Counter(r.get("aheadSrc", "") for r in live if halt(r) and r["by"] == "ahead")),
        "no_frames_rows": sum(1 for r in rows if r["status"] == "no_frames"),
        "stale_rows": sum(1 for r in rows if r["stale"]),
    }
    chk = res["checks"]

    # Invariant for every walk: no walking instruction without the validator's permission.
    bad = [r for r in live if r["guide"] in MOVE and not r["mayMove"]]
    chk.append(("never walking advice without permission", not bad, f"{len(bad)} rows"))
    chk.append(("frame rate >= 20 fps", res["stats"]["fps"] >= 20, f"{res['stats']['fps']} fps"))

    # Approach tests (S1 wall, S5 person, S9 chair) only count once the start area is clear:
    # nothing within 1.5 m ahead and no HALT for 1 s. A walk started next to something
    # (phone: S5 began 0.61 m from an object) is "invalid start", not a safety failure.
    def clear_row(r):
        return r["verdict"] != 2 and (r["ahead"] is None or r["ahead"] < 0 or r["ahead"] > 1.5) and \
            (r["vdist"] is None or r["vdist"] < 0 or r["vdist"] > 1.5)
    def approach_rows():
        run_start = None
        for idx, r in enumerate(live):
            if clear_row(r):
                if run_start is None:
                    run_start = idx
                if r["ms"] - live[run_start]["ms"] >= 1000:
                    return live[run_start:]
            else:
                run_start = None
        return None

    fh = first(live, halt)
    if sid in ("S1", "S5", "S9"):
        appr = approach_rows()
        chk.append(("valid start (clear for 1 s before the approach)", appr is not None,
                    "" if appr is not None else "something within 1.5 m from the start - restart facing open space"))
        if appr is None:
            res["notes"].append("invalid start: the scenario checks were skipped")
            return res
        res["notes"].append(f"approach counted from {appr[0]['ms'] / 1000:.1f} s")
        live = appr
        fh = first(live, halt)
    def spoke_after(t_ms, keys, within_ms):
        return any(t_ms <= t <= t_ms + within_ms and k in keys for t, k, _ in said)

    if sid == "S1":   # wall approach
        # STOP rule: an object within 0.5 m ahead (0.3 m minimum: the user still stops in time).
        ok = fh is not None and fh["by"] in ("ahead", "obstacle", "head") and (fh["vdist"] or 0) >= 0.3
        chk.append(("STOP at 0.3-0.5 m from the wall (not closer)", ok,
                    f"first HALT by {fh['by']} at {fh['vdist']} m" if fh else "no HALT"))
        if fh:
            chk.append(("STOP spoken within 0.6 s", spoke_after(fh["ms"], HALT_SAY, 600), ""))
        # Within 0.7 m only a detour (bear / turn away) may be advised, never "go straight".
        close = [r for r in live if r["ahead"] is not None and 0 < r["ahead"] <= 0.7 and r["mayMove"]
                 and r["guide"] not in TURNS]
        chk.append(("no straight-on advice with an object <= 0.7 m ahead", not close, f"{len(close)} rows"))
    elif sid == "S2":   # clear corridor
        st = res["stats"]
        chk.append(("HALT <= 10 % of the time", st["halt_%"] <= 10, f"{st['halt_%']} %"))
        chk.append(("walking advice allowed >= 50 %", st["may_move_%"] >= 50, f"{st['may_move_%']} %"))
        chk.append(("<= 15 instruction changes / min", st["changes_per_min"] <= 15, f"{st['changes_per_min']}"))
        sf = sum(1 for r in live if r["health"] == 2)
        chk.append(("no camera faults", sf == 0, f"{sf} rows"))
    elif sid == "S3":   # doorway
        halt_s = sum(1 for r in live if halt(r)) * dur / len(live)
        chk.append(("HALT <= 3 s in total", halt_s <= 3, f"{halt_s:.1f} s"))
        bl = frac(live, lambda r: r["planBlocked"])
        chk.append(("'no clear way' <= 30 % of the time", bl <= 0.3, f"{100 * bl:.0f} %"))
        chk.append(("walking advice allowed >= 40 %", res["stats"]["may_move_%"] >= 40, f"{res['stats']['may_move_%']} %"))
    elif sid == "S4":   # corner
        tr = [r for r in live if r["guide"] in TURNS]
        chk.append(("a turn / bear instruction was given", bool(tr), f"{len(tr)} rows"))
        chk.append(("<= 20 instruction changes / min", res["stats"]["changes_per_min"] <= 20,
                    f"{res['stats']['changes_per_min']}"))
        chk.append(("HALT <= 20 % of the time", res["stats"]["halt_%"] <= 20, f"{res['stats']['halt_%']} %"))
    elif sid == "S5":   # person approaching
        hit = first(live, lambda r: r["verdict"] >= 1 and r["by"] in ("moving", "ahead"))
        chk.append(("person detected (CAUTION / STOP, moving or ahead)", hit is not None,
                    f"at {hit['ms'] / 1000:.1f} s by {hit['by']}" if hit else "never"))
        hs = first(live, lambda r: halt(r) and r["by"] in ("moving", "ahead"))
        chk.append(("STOP before the person is within 0.3 m", hs is not None and (hs["vdist"] or 9) >= 0.3 or
                    (hs is not None and hs["by"] == "moving"),
                    f"{hs['by']} at {hs['vdist']} m" if hs else "no STOP"))
    elif sid == "S6":   # step down / stairs
        hd = first(live, lambda r: halt(r) and r["by"] in ("dropoff", "stairs"))
        chk.append(("STOP for the step at >= 0.8 m", hd is not None and (hd["vdist"] or 0) >= 0.8,
                    f"{hd['by']} at {hd['vdist']} m" if hd else "no drop-off / stairs STOP"))
    elif sid == "S7":   # camera covered
        cov = first(live, lambda r: r["depthValid"] is not None and r["depthValid"] < 0.10)
        if cov is None:
            chk.append(("camera was covered (depth < 10 %)", False, "never covered"))
        else:
            sf = first([r for r in live if r["ms"] >= cov["ms"]], lambda r: halt(r) and r["by"] == "sensor")
            lat = (sf["ms"] - cov["ms"]) / 1000 if sf else None
            chk.append(("camera STOP within 1.0 s of covering", sf is not None and lat <= 1.0,
                        f"{lat:.2f} s" if lat is not None else "no camera STOP"))
            chk.append(("'camera blocked' spoken within 1.5 s", spoke_after(cov["ms"], {"sensor_fault"}, 1500), ""))
            clear = first([r for r in live if sf and r["ms"] > sf["ms"]], lambda r: r["health"] == 0)
            chk.append(("recovers after uncovering", clear is not None, ""))
    elif sid == "S8":   # standing still
        chk.append(("no STOP in open space", not any(halt(r) for r in live),
                    f"{res['stats']['halt_%']} % HALT ({res['stats']['halted_by']})"))
        per30 = len(said) * 30 / dur if dur > 0 else 0
        chk.append(("<= 6 spoken messages per 30 s", per30 <= 6, f"{per30:.1f}"))
    elif sid == "S9":   # chair in path
        fh9 = first(live, lambda r: halt(r))
        detour = first(live, lambda r: r["guide"] in TURNS and (r["ahead"] is None or r["ahead"] < 0 or r["ahead"] > 1.0))
        ok = (fh9 is not None and (fh9["vdist"] or 0) >= 0.3) or detour is not None
        chk.append(("STOP at 0.3-0.5 m or a detour instruction", ok,
                    (f"HALT {fh9['by']} at {fh9['vdist']} m" if fh9 else "no HALT") +
                    (f"; detour '{detour['guide']}' at {detour['ms'] / 1000:.1f} s" if detour else "")))
        close = [r for r in live if r["ahead"] is not None and 0 < r["ahead"] <= 0.7 and r["mayMove"]
                 and r["guide"] not in TURNS]
        chk.append(("no straight-on advice with the chair <= 0.7 m ahead", not close, f"{len(close)} rows"))
    else:
        res["notes"].append("unknown scenario: only the common checks were run")
    return res


def verdict(r):
    if any(n.startswith("invalid start") for n in r["notes"]):
        return "INVALID"
    return "PASS" if all(c[1] for c in r["checks"]) else "FAIL"


def report(results, out_md):
    lines = ["# HearSight Phase 7 validation report", ""]
    total = sum(len(r["checks"]) for r in results)
    passed = sum(1 for r in results for _, ok, _ in r["checks"] if ok)
    lines += [f"**{passed} / {total} checks passed** across {len(results)} walks.", "",
              "| Walk | Scenario | Result | Duration | FPS | HALT % | Changes/min | Speech/min |",
              "|---|---|---|---|---|---|---|---|"]
    for r in results:
        st = r.get("stats", {})
        lines.append(f"| {r['file']} | {r['scenario']} {r['label']} | {verdict(r)} | "
                     f"{st.get('duration_s', '-')} s | {st.get('fps', '-')} | {st.get('halt_%', '-')} | "
                     f"{st.get('changes_per_min', '-')} | {st.get('speech_per_min', '-')} |")
    for r in results:
        lines += ["", f"## {r['scenario']} {r['label']} ({r['file']})", ""]
        for name, ok, detail in r["checks"]:
            lines.append(f"- {'PASS' if ok else '**FAIL**'}: {name}" + (f" ({detail})" if detail else ""))
        if r.get("stats"):
            lines.append(f"- HALT decided by: {r['stats']['halted_by'] or 'none'}")
            if r['stats'].get('ahead_from'):
                lines.append(f"- 'object ahead' came from: {r['stats']['ahead_from']}")
        for n in r["notes"]:
            lines.append(f"- note: {n}")
    with open(out_md, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return passed, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*")
    ap.add_argument("--pull", action="store_true", help="adb pull the phone's validation files first")
    ap.add_argument("--device", help="adb serial / ip:port")
    ap.add_argument("--out", default="validation_runs", help="where pulled files and the report go")
    a = ap.parse_args()
    files = list(a.files)
    out_dir = a.out
    if a.pull:
        os.makedirs(out_dir, exist_ok=True)
        cmd = ["adb"] + (["-s", a.device] if a.device else []) + ["pull", PHONE_DIR, out_dir]
        subprocess.run(cmd, check=True)
        d = os.path.join(out_dir, "validation")
        files += sorted(os.path.join(d, x) for x in os.listdir(d) if x.endswith(".csv"))
    elif files:
        out_dir = os.path.dirname(os.path.abspath(files[0]))
    if not files:
        ap.error("no files (give CSV files or --pull)")
    results = [score(p) for p in files]
    md = os.path.join(out_dir, "validation_report.md")
    passed, total = report(results, md)
    for r in results:
        print(f"{verdict(r):7} {r['scenario']:3} {r['label']:22} {r['file']}")
        for name, cok, detail in r["checks"]:
            if not cok:
                print(f"      FAIL {name} ({detail})")
    print(f"{passed}/{total} checks passed -> {md}")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
