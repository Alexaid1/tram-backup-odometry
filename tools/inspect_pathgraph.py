import json, glob, numpy as np
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
for f in sorted(ROOT.glob("Pathgraph*/*.json")):
    d = json.load(open(f, encoding="utf-8"))
    print("==", f.name, list(d.keys()))
    for k, v in d.items():
        if isinstance(v, list):
            print(" key", k, "len", len(v), "elem", type(v[0]).__name__,
                  list(v[0].keys()) if isinstance(v[0], dict) else "")
            if k != "points":
                for i, e in enumerate(v[:50]):
                    if isinstance(e, dict):
                        print("   [%d]" % i, {kk: (f"list[{len(vv)}] {vv[:3]}..{vv[-3:]}" if isinstance(vv, list) else vv) for kk, vv in e.items()})
        else:
            print(" key", k, str(v)[:300])
    P = np.array([[p["x"], p["y"], p["z"]] for p in d["points"]])
    seg = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)
    print(" points", len(P), "x", P[:, 0].min(), P[:, 0].max(), "y", P[:, 1].min(), P[:, 1].max(), "z", P[:, 2].min(), P[:, 2].max())
    print(" step p50 %.3f min %.3f max %.3f total %.1f m" % (np.median(seg), seg.min(), seg.max(), seg.sum()))
    big = np.where(seg > 3)[0]
    print(" jumps>3m at", big[:30], np.round(seg[big][:30], 1))
    t = np.array([p["tang"] for p in d["points"]])
    h = np.arctan2(np.diff(P[:, 1]), np.diff(P[:, 0]))
    dd = np.angle(np.exp(1j * (t[:-1] - h)))
    print(" tang-heading diff median", np.median(np.abs(dd)), "p95", np.percentile(np.abs(dd), 95))
