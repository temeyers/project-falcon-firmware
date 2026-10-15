#!/usr/bin/env python3
"""Turn a `log on` serial capture into labeled CSVs.
Usage: python3 tools/split_log.py washer_run1.csv [out_dir]
Input lines (anything else, e.g. [HTTP]/[ALARM] lines, is ignored):
  L,t_ms,ax,ay,az,gx,gy,gz,run,open,armed,rms   sample
  M,t_ms,<label>                               mark (fill/wash/rinse/spin/drain/idle/door/...)
  E,t_ms,<event>,<arg>                         firmware event (cycle_start, cycle_end, cycle_stalled, ...)
Output: <out>/labeled.csv (every sample + label + last_event), <out>/by_label/<label>.csv, <out>/events.csv, summary on stdout.
"""
import csv, os, sys, collections
COLS = ["t_ms","ax","ay","az","gx","gy","gz","run","open","armed","rms"]
def main():
    src = sys.argv[1]; out = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + "_split"
    os.makedirs(os.path.join(out, "by_label"), exist_ok=True)
    label, last_ev = "unlabeled", ""
    rows, events = [], []
    for raw in open(src, errors="ignore"):
        line = raw.strip()
        for p in ("L,", "M,", "E,"):          # tolerate monitor prefixes/timestamps before the record
            j = line.find(p)
            if j >= 0 and (j == 0 or not line[j-1].isalnum()): line = line[j:]; break
        f = line.split(",")
        if f[0] == "L" and len(f) == 12:
            try: vals = [float(x) for x in f[1:]]
            except ValueError: continue
            rows.append(dict(zip(COLS, f[1:]), label=label, last_event=last_ev))
        elif f[0] == "M" and len(f) >= 3:
            label = ",".join(f[2:]).strip().lower().replace(" ", "_") or "unlabeled"
        elif f[0] == "E" and len(f) >= 3:
            last_ev = f[2]; events.append({"t_ms": f[1], "event": f[2], "arg": f[3] if len(f) > 3 else "", "label": label})
    if not rows: sys.exit("no L, sample lines found - was `log on` enabled?")
    fields = COLS + ["label", "last_event"]
    with open(os.path.join(out, "labeled.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields); w.writeheader(); w.writerows(rows)
    groups = collections.defaultdict(list)
    for r in rows: groups[r["label"]].append(r)
    for lab, rs in groups.items():
        with open(os.path.join(out, "by_label", f"{lab}.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields); w.writeheader(); w.writerows(rs)
    with open(os.path.join(out, "events.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["t_ms","event","arg","label"]); w.writeheader(); w.writerows(events)
    t0, t1 = float(rows[0]["t_ms"]), float(rows[-1]["t_ms"])
    print(f"{len(rows)} samples over {(t1-t0)/1000:.0f} s -> {out}")
    for lab, rs in groups.items():
        rms = sum(float(r["rms"]) for r in rs) / len(rs)
        print(f"  {lab:12s} {len(rs):7d} samples  mean rms {rms:.4f} g")
    print(f"  events: {', '.join(e['event'] for e in events) or 'none'}")
if __name__ == "__main__": main()
