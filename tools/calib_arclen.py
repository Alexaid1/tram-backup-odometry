"""Калибровка масштаба одометрии относительно длины дуги pathgraph.

Для каждого прогона: проекция эталонных точек (kinematic_state или GNSS master) на
ближайшую полилинию -> s_ref(t). Сравниваем приращение s_ref на участке по карте с
интегралом скорости тележек (v_kmh/3.6) за то же время.
"""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
C = ROOT / "tools" / "_cache"
paths = []
for f in sorted(ROOT.glob("Pathgraph*/*.json")):
    d = json.load(open(f, encoding="utf-8"))
    paths.append(np.array([[p["x"], p["y"]] for p in d["points"]]))


def project(P, xy):
    """Return (s, dist) of projection of xy onto polyline P (1 m step)."""
    seg = np.diff(P, axis=0); L = np.linalg.norm(seg, axis=1); cum = np.r_[0, np.cumsum(L)]
    out_s = np.empty(len(xy)); out_d = np.empty(len(xy))
    for i, q in enumerate(xy):
        w = q - P[:-1]
        tt = np.clip((w * seg).sum(1) / (L * L), 0, 1)
        pr = P[:-1] + seg * tt[:, None]
        dd = np.linalg.norm(pr - q, axis=1); j = dd.argmin()
        out_s[i] = cum[j] + tt[j] * L[j]; out_d[i] = dd[j]
    return out_s, out_d


def odo_dist(t_query, z):
    ft, fv = z["front_h"], z["front_v"][:, 0] / 3.6
    rt, rv = z["rear_h"], z["rear_v"][:, 0] / 3.6
    t = np.arange(t_query.min(), t_query.max(), 0.02)
    v = 0.5 * (np.interp(t, ft, fv) + np.interp(t, rt, rv))
    S = np.r_[0, np.cumsum(v[:-1] * 0.02)]
    return np.interp(t_query, t, S)


res = []
for f in sorted(C.glob("*.npz")):
    z = np.load(f)
    if "ref_v" in z:
        t = z["ref_h"][::25]; xy = z["ref_v"][::25, :2]; src = "ref"
    elif "mfix_xy" in z and len(z["mfix_xy"]) > 3000:
        m = z["mfix_v"][:, 3] == 2
        if m.sum() < 1000:
            continue
        t = z["mfix_h"][m][::5]; xy = z["mfix_xy"][m][::5]; src = "gnss"
    else:
        continue
    fr = np.abs(np.interp(t, z["front_h"], z["front_v"][:, 0]) - np.interp(t, z["rear_h"], z["rear_v"][:, 0]))
    for k, P in enumerate(paths):
        s, d = project(P, xy)
        on = (d < 1.5) & (s > 5) & (s < (len(P) - 6))
        if on.sum() < 100:
            continue
        idx = np.where(on)[0]
        # longest contiguous stretch in time along this path
        a, b = idx[0], idx[-1]
        if s[b] - s[a] < 1000 or np.any(fr[a:b] > 3.0 / 1):  # skip runs with big faults
            continue
        od = odo_dist(t[[a, b]], z)
        ratio = (s[b] - s[a]) / (od[1] - od[0])
        res.append((f.stem, src, k, s[b] - s[a], ratio))
        print(f"{f.stem} {src} path{k}: ds_map={s[b]-s[a]:.1f} ds_odo={od[1]-od[0]:.1f} ratio={ratio:.5f}")
r = np.array([x[4] for x in res])
print(f"\nN={len(r)} ratio median={np.median(r):.5f} mean={r.mean():.5f} std={r.std():.5f} p10={np.percentile(r,10):.5f} p90={np.percentile(r,90):.5f}")
for k in (0, 1):
    rk = np.array([x[4] for x in res if x[2] == k]); print(f" path{k}: median {np.median(rk):.5f} n={len(rk)}")
cal = json.load(open(ROOT / "tools/calib.json"))
cal["arc_scale"] = float(np.median(r))
json.dump(cal, open(ROOT / "tools/calib.json", "w"), indent=2)
