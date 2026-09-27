"""Сводка по всем прогонам датасета + общий рисунок траекторий."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
C = ROOT / "tools" / "_cache"
paths = {}
for f in sorted(ROOT.glob("Pathgraph*/*.json")):
    d = json.load(open(f, encoding="utf-8"))
    paths[f.stem] = np.array([[p["x"], p["y"]] for p in d["points"]])
allP = np.concatenate(list(paths.values()))


def dist_to_map(xy):
    out = np.empty(len(xy))
    for i in range(0, len(xy), 2000):
        blk = xy[i:i + 2000]
        d = np.sqrt(((blk[:, None, :] - allP[None, ::2, :]) ** 2).sum(-1)).min(1)
        out[i:i + 2000] = d
    return out


rows = []
fig, ax = plt.subplots(figsize=(14, 8))
for name, P in paths.items():
    ax.plot(P[:, 0], P[:, 1], "k-", lw=2, zorder=5)
for f in sorted(C.glob("*.npz")):
    z = np.load(f)
    if "front_v" not in z or len(z["front_v"]) < 5:
        rows.append((f.stem, 0, 0, 0, 0, np.nan, np.nan, np.nan, np.nan, np.nan)); continue
    ft, fv = z["front_h"], z["front_v"][:, 0]
    rt, rv = z["rear_h"], z["rear_v"][:, 0]
    dur = ft[-1] - ft[0]
    dist = np.sum(fv[:-1] / 3.6 * np.clip(np.diff(ft), 0, 1))
    fr = np.abs(fv - np.interp(ft, rt, rv)) / 3.6
    gcov = gspd_err = offmap = np.nan
    if "mfix_xy" in z and len(z["mfix_xy"]) > 2:
        gt = z["mfix_h"]
        gcov = len(gt) / 10.0 / max(dur, 1)
        dm = dist_to_map(z["mfix_xy"])
        offmap = np.mean(dm > 5)
        ax.plot(z["mfix_xy"][:, 0], z["mfix_xy"][:, 1], ".", ms=0.5, alpha=0.3)
    if "mvel_v" in z and len(z["mvel_v"]) > 2:
        gs = np.hypot(z["mvel_v"][:, 0], z["mvel_v"][:, 1])
        vo = np.interp(z["mvel_h"], ft, fv) / 3.6
        gspd_err = np.sqrt(np.mean((vo - gs) ** 2))
    nstop = np.mean(fv < 0.1)
    rows.append((f.stem, dur, dist, len(ft) / max(dur, 1), gcov, offmap, gspd_err,
                 np.percentile(fr, 99), fr.max(), nstop))
ax.set_aspect("equal"); ax.set_title("GNSS master (all runs) vs pathgraph (black)")
fig.savefig(ROOT / "tools/_scan_map.png", dpi=110)

print(f"{'bag':16s} {'dur,s':>7s} {'dist,m':>8s} {'odoHz':>5s} {'gnssCov':>7s} {'offmap':>6s} "
      f"{'vErrRMS':>7s} {'FRp99':>6s} {'FRmax':>6s} {'stop':>5s}")
for r in rows:
    print(f"{r[0]:16s} {r[1]:7.0f} {r[2]:8.0f} {r[3]:5.1f} {r[4]:7.2f} {r[5]:6.2f} {r[6]:7.3f} "
          f"{r[7]:6.2f} {r[8]:6.2f} {r[9]:5.2f}")
A = np.array([r[1:] for r in rows], float)
print("\nTOTAL dur h=%.1f dist km=%.0f; median gnssCov=%.2f offmap=%.2f vErr=%.3f" % (
    np.nansum(A[:, 0]) / 3600, np.nansum(A[:, 1]) / 1000, np.nanmedian(A[:, 3]), np.nanmedian(A[:, 4]), np.nanmedian(A[:, 5])))
np.save(ROOT / "tools/_scan.npy", np.array(rows, dtype=object), allow_pickle=True)
