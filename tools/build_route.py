"""Офлайн-построение замкнутого маршрута: pathgraph + разворотные петли.

Pathgraph содержит только основную линию (две полилинии по направлениям). Разворотные
петли на конечных (и подход к петле у Щукинской) в нём отсутствуют. Их геометрия
восстанавливается ОФЛАЙН по GNSS-трекам обучающего датасета (усреднение нескольких
прогонов), затем всё сшивается в одну замкнутую полилинию с шагом 1 м:

    A (таллинская->щукинская) + ext_NE + B (щукинская->таллинская) + ext_SW  -> A ...

Результат пишется в формате, совместимом с pathgraph (points[x,y,z,tang,curv], paths)
+ служебное поле "segments" (какие участки из pathgraph, какие — восстановленные).
В рабочем режиме GNSS не используется: это статическая карта.
"""
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import gaussian_filter1d

ROOT = Path(__file__).resolve().parents[1]
C = ROOT / "tools" / "_cache"
MASTER_X = -9.873       # master antenna in base_link
ANT_Z = 3.0
DZ = -0.14              # map_z - gnss_alt (fit_geo)

pg = {}
for f in sorted(ROOT.glob("Pathgraph*/*.json")):
    d = json.load(open(f, encoding="utf-8"))
    pg[f.stem] = np.array([[p["x"], p["y"], p["z"]] for p in d["points"]])
A = pg["таллинская - щукинская"]
B = pg["щукинская - таллинская"]


def nearest(P, q):
    d = np.linalg.norm(P[:, :2] - q[:2], axis=1); j = d.argmin(); return j, d[j]


def base_track(z):
    m = z["mfix_v"][:, 3] == 2
    t = z["mfix_h"][m]; xy = z["mfix_xy"][m]; alt = z["mfix_v"][m, 2]
    o = np.argsort(t); t, xy, alt = t[o], xy[o], alt[o]
    # heading from motion (central difference over ~±1 s)
    n = len(t); h = np.full(n, np.nan)
    for i in range(n):
        a = np.searchsorted(t, t[i] - 1.0); b = min(np.searchsorted(t, t[i] + 1.0), n - 1)
        dxy = xy[b] - xy[a]
        if np.hypot(*dxy) > 1.0:
            h[i] = np.arctan2(dxy[1], dxy[0])
    ok = ~np.isnan(h)
    base = xy[ok] + (-MASTER_X) * np.stack([np.cos(h[ok]), np.sin(h[ok])], -1)
    bz = alt[ok] + DZ - ANT_Z
    return t[ok], np.column_stack([base, bz])


def contiguous(t, X, a, b, max_step=5.0):
    """No spatial holes (stops remove points but do not move the tram)."""
    return b > a and np.all(np.linalg.norm(np.diff(X[a:b + 1, :2], axis=0), axis=1) < max_step)


def approach(track, P_end):
    """From passing the end of path P_end to the end of the run (terminal stop)."""
    t, X = track
    d = np.linalg.norm(X[:, :2] - P_end[-1, :2], axis=1)
    idx = np.where(d < 1.5)[0]
    if not len(idx):
        return None
    a = idx[-1]
    return X[a:] if contiguous(t, X, a, len(X) - 1) and len(X) - a > 20 else None


def departure(track, P_start):
    """From the beginning of the run (terminal stop) to joining the start of P_start."""
    t, X = track
    d = np.linalg.norm(X[:, :2] - P_start[0, :2], axis=1)
    idx = np.where(d < 1.5)[0]
    if not len(idx):
        return None
    b = idx[0]
    return X[:b + 1] if contiguous(t, X, 0, b) and b > 20 else None


def resample_1m(seg):
    L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(seg[:, :2], axis=0), axis=1))]
    keep = np.r_[True, np.diff(L) > 1e-3]
    L, seg = L[keep], seg[keep]
    q = np.arange(0, L[-1], 1.0)
    return np.column_stack([np.interp(q, L, seg[:, k]) for k in range(3)])


def typical(segs):
    """Pick the segment closest (median point distance) to all others; smooth it."""
    segs = [resample_1m(s) for s in segs]
    Ls = np.array([len(s) for s in segs])
    segs = [s for s in segs if len(s) > 0.5 * np.median(Ls)]
    score = []
    for i, s in enumerate(segs):
        ds = []
        for j, o in enumerate(segs[:40]):
            if i == j:
                continue
            d = np.sqrt(((s[::5, None, :2] - o[None, ::2, :2]) ** 2).sum(-1)).min(1)
            ds.append(np.median(d))
        score.append(np.median(ds) - 1e-4 * len(s))  # prefer long & consistent
    k = int(np.argmin(score))
    S = segs[k].copy()
    for c in range(3):
        S[:, c] = gaussian_filter1d(S[:, c], 1.5, mode="nearest")
    return S, len(segs), score[k]


def longest(segs):
    """Longest plausible segment (covers the whole loop); drop depot excursions (> p90)."""
    segs = [resample_1m(s) for s in segs]
    Ls = np.array([len(s) for s in segs])
    ok = [s for s in segs if len(s) <= np.percentile(Ls, 90)]
    S = max(ok, key=len).copy()
    for c in range(3):
        S[:, c] = gaussian_filter1d(S[:, c], 1.5, mode="nearest")
    return S


def splice(app, dep, P_in, P_out):
    """approach (from P_in end) + departure (to P_out start) -> connecting polyline.

    Join on the SAME track (inbound track before the loop): earliest departure point
    within 0.8 m of the approach (parallel track is ~3.5 m away)."""
    d = np.linalg.norm(app[:, None, :2] - dep[None, :, :2], axis=2)
    cand = np.argwhere(d < 0.8)
    if len(cand):
        i, j = cand[np.argmin(cand[:, 1])]
    else:
        i, j = np.unravel_index(np.argmin(d), d.shape)
    # prefer joining where approach ends / departure starts (the stop)
    conn = np.vstack([app[:i + 1], dep[j + 1:]])
    conn = resample_1m(conn)
    # blend endpoints into pathgraph ends for continuity
    n = len(conn); w = np.clip(np.arange(n) / 10.0, 0, 1)
    conn = conn + (1 - w)[:, None] * (P_in[-1] - conn[0]) + (1 - w[::-1])[:, None] * (P_out[0] - conn[-1])
    return conn, d[i, j]


import sys
if len(sys.argv) > 1:  # диагностика одного прогона
    z = np.load(C / f"{sys.argv[1]}.npz"); t, X = base_track(z)
    print(len(t), "status2 frac", (z["mfix_v"][:, 3] == 2).mean(), "max gap", np.diff(t).max(),
          "gaps>1s", int((np.diff(t) > 1).sum()))
    for nm, pt in (("A_end", A[-1]), ("B_start", B[0]), ("B_end", B[-1]), ("A_start", A[0])):
        d = np.linalg.norm(X[:, :2] - pt[:2], axis=1)
        print(nm, "min dist %.2f at t=%.0f" % (d.min(), t[d.argmin()] - t[0]))
    sys.exit()

appNE, depNE, appSW, depSW = [], [], [], []
for f in sorted(C.glob("*.npz")):
    z = np.load(f)
    if "mfix_xy" not in z or len(z["mfix_xy"]) < 3000 or "ref_v" in z:
        continue
    if (z["mfix_v"][:, 3] == 2).mean() < 0.3:
        continue
    tr = base_track(z)
    for lst, fn, P in ((appNE, approach, A), (appSW, approach, B), (depNE, departure, B), (depSW, departure, A)):
        s = fn(tr, P)
        if s is not None:
            lst.append(s)
print("segments: appNE", len(appNE), "depNE", len(depNE), "appSW", len(appSW), "depSW", len(depSW))
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, axs = plt.subplots(1, 2, figsize=(16, 8))
for ax, apps, deps in ((axs[0], appNE, depNE), (axs[1], appSW, depSW)):
    for s in apps:
        ax.plot(s[:, 0], s[:, 1], "b-", lw=0.6, alpha=0.5); ax.plot(s[-1, 0], s[-1, 1], "bo", ms=3)
    for s in deps:
        ax.plot(s[:, 0], s[:, 1], "r-", lw=0.6, alpha=0.5); ax.plot(s[0, 0], s[0, 1], "r^", ms=3)
    ax.plot(A[:, 0], A[:, 1], "k-", lw=0.5); ax.plot(B[:, 0], B[:, 1], "k--", lw=0.5)
    allp = np.vstack(apps + deps); ax.set_xlim(allp[:, 0].min() - 10, allp[:, 0].max() + 10)
    ax.set_ylim(allp[:, 1].min() - 10, allp[:, 1].max() + 10); ax.set_aspect("equal"); ax.grid()
fig.savefig(ROOT / "tools/_route_segs.png", dpi=80)
def heading_at(S, k, back=True):
    a, b = (max(k - 8, 0), k) if back else (k, min(k + 8, len(S) - 1))
    d = S[b, :2] - S[a, :2]
    return d / max(np.linalg.norm(d), 1e-6)


def best_pair(apps, deps, P_in, P_out):
    """Choose (approach, departure) where the departure continues the approach on the
    same track in the same direction (the terminal stop between two consecutive runs)."""
    apps = [resample_1m(s) for s in apps]; deps = [resample_1m(s) for s in deps]
    La = np.array([len(s) for s in apps]); Ld = np.array([len(s) for s in deps])
    best = None
    for a in apps:
        if len(a) > 2000 or len(a) < 10:
            continue
        h = heading_at(a, len(a) - 1)
        for dp in deps:
            if len(dp) > 2000 or len(dp) < 10:
                continue
            g = dp[0, :2] - a[-1, :2]
            along = g @ h; cross = abs(g[0] * h[1] - g[1] * h[0])
            hd = heading_at(dp, 0, back=False)
            if not (-15 <= along <= 40 and cross < 1.0 and h @ hd > 0.96):
                continue
            L = len(a) + len(dp)
            if best is None or L > best[0]:
                best = (L, a, dp, along, cross)
    if best is None:
        diag = []
        for a in apps:
            h = heading_at(a, len(a) - 1)
            for dp in deps:
                g = dp[0, :2] - a[-1, :2]
                diag.append((abs(g[0] * h[1] - g[1] * h[0]) + 0.05 * abs(g @ h), g @ h,
                             abs(g[0] * h[1] - g[1] * h[0]), h @ heading_at(dp, 0, False), len(a), len(dp)))
        for r in sorted(diag)[:8]:
            print("   cand: along %.1f cross %.2f cos %.3f lenA %d lenD %d" % r[1:])
        raise SystemExit("no pair")
    L, a, dp, along, cross = best
    if along < 0:  # overlap: trim approach at the point closest to departure start
        k = np.argmin(np.linalg.norm(a[:, :2] - dp[0, :2], axis=1)); a = a[:k]
    conn = resample_1m(np.vstack([a, dp]))
    for c in range(3):
        conn[:, c] = gaussian_filter1d(conn[:, c], 1.5, mode="nearest")
    n = len(conn); w = np.clip(np.arange(n) / 10.0, 0, 1)
    conn = conn + (1 - w)[:, None] * (P_in[-1] - conn[0]) + (1 - w[::-1])[:, None] * (P_out[0] - conn[-1])
    return conn, along, cross


ne, al1, cr1 = best_pair(appNE, depNE, A, B)
sw, al2, cr2 = best_pair(appSW, depSW, B, A)
print(f"NE splice: gap along {al1:.1f} m cross {cr1:.2f} m; SW splice: along {al2:.1f} cross {cr2:.2f}")
gne, gsw = cr1, cr2
nne, nsw = len(appNE) + len(depNE), len(appSW) + len(depSW)
for ax, M in ((axs[0], ne), (axs[1], sw)):
    ax.plot(M[:, 0], M[:, 1], "k-", lw=2.5, alpha=0.6)
fig.savefig(ROOT / "tools/_route_segs.png", dpi=80)
print(f"NE connector: {len(ne)} m (splice gap {gne:.2f} m); SW connector: {len(sw)} m (gap {gsw:.2f} m)")
segsNE = [(0, s) for s in appNE + depNE]; segsSW = [(0, s) for s in appSW + depSW]

parts = [("pathgraph:таллинская - щукинская", A), ("reconstructed:loop_NE", ne[1:-1]),
         ("pathgraph:щукинская - таллинская", B), ("reconstructed:loop_SW", sw[1:-1])]
pts = []; segments = []
for name, P in parts:
    segments.append({"name": name, "first": len(pts), "last": len(pts) + len(P) - 1})
    pts.extend(P.tolist())
P = np.array(pts)
# tangent & curvature on closed polyline
d_next = np.roll(P, -1, 0) - P
tang = np.arctan2(d_next[:, 1], d_next[:, 0])
dt = np.angle(np.exp(1j * (np.roll(tang, -1) - tang)))
ds = np.linalg.norm(d_next[:, :2], axis=1)
curv = dt / np.maximum(ds, 1e-3)
print("closed route: points", len(P), "length %.1f m" % ds.sum(), "max step %.2f" % ds.max())
out = {
    "points": [{"x": float(p[0]), "y": float(p[1]), "z": float(p[2]), "tang": float(a), "curv": float(c)}
               for p, a, c in zip(P, tang, curv)],
    "paths": [{"ext_id": "closed_route", "point_indices": list(range(len(P))), "closed": True}],
    "segments": segments,
    "note": "pathgraph + terminal loops reconstructed offline from training-set GNSS tracks "
            f"(NE: {nne} runs, SW: {nsw} runs). Static map; GNSS is not used at runtime.",
}
dst = ROOT / "ros2_ws/src/tram_odometry/maps"
dst.mkdir(parents=True, exist_ok=True)
json.dump(out, open(dst / "route_closed.json", "w", encoding="utf-8"), ensure_ascii=False)

import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, axs = plt.subplots(1, 2, figsize=(14, 6))
for ax, (seg, M, c) in zip(axs, ((segsNE, ne, A[-1]), (segsSW, sw, B[-1]))):
    for L, s in seg[:30]:
        ax.plot(s[:, 0], s[:, 1], "-", lw=0.5, alpha=0.4)
    ax.plot(M[:, 0], M[:, 1], "k-", lw=2); ax.plot(A[:, 0], A[:, 1], "b-", lw=1); ax.plot(B[:, 0], B[:, 1], "r-", lw=1)
    ax.set_xlim(M[:, 0].min() - 30, M[:, 0].max() + 30); ax.set_ylim(M[:, 1].min() - 30, M[:, 1].max() + 30)
    ax.set_aspect("equal"); ax.grid()
fig.savefig(ROOT / "tools/_route_ext.png", dpi=90)

