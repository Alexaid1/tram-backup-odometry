"""Offline map of repeatable stop locations (stations) along the closed route.

For every training run with GNSS: find standstill episodes (both bogies ~0 for > 5 s after
moving), take the GNSS base_link position during the stop, project it onto the closed route
near the track branch matching the direction of motion, and collect arc positions s.
Clusters with small spread and high attendance are stations (traffic-light stops scatter
and are rejected). Output: maps/stations.json  [{s, sigma, count, passes}].
"""
import json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from evaluate import Route, reference, PKG, C

import argparse
ap = argparse.ArgumentParser()
ap.add_argument("--fold", type=int, default=None, help="use only runs with index %% 2 == fold")
ap.add_argument("--out", default=str(PKG / "maps/stations.json"))
ARGS = ap.parse_args()
route = Route(PKG / "maps/route_closed.json")
run_index = 0
L = route.total
stops = []   # (run, s)
passes = np.zeros(int(L) + 1)
seen = set()
for f in sorted(C.glob("*.npz")):
    z = np.load(f)
    if "front_v" not in z or len(z["front_v"]) < 3000 or "ref_v" in z:
        continue  # exclude the check bag (it is the jury-like test)
    key = (len(z["front_v"]), round(float(z["front_v"][:, 0].sum()), 1))
    if key in seen:
        continue
    seen.add(key)
    run_index += 1
    if ARGS.fold is not None and run_index % 2 != ARGS.fold:
        continue
    ref = reference(z)
    if ref is None or len(ref["t"]) < 3000:
        continue
    t = z["front_h"]; v = 0.5 * (z["front_v"][:, 0] + np.interp(t, z["rear_h"], z["rear_v"][:, 0])) / 3.6
    still = v < 0.05
    # standstill episodes
    edges = np.flatnonzero(np.diff(still.astype(int)))
    starts = edges[still[edges + 1]] + 1 if len(edges) else []
    s_prev = None
    # route position of the reference along time, branch-consistent: project with heading
    # continuity by tracking the previous s
    sub = np.arange(0, len(ref["t"]), 10)
    xy = np.column_stack([ref["x"][sub], ref["y"][sub]])
    # initial branch: nearest segment whose direction agrees with the motion heading
    k0 = next((k for k in range(len(xy) - 5) if np.hypot(*(xy[k + 5] - xy[k])) > 5.0), None)
    if k0 is None:
        continue
    hd = xy[k0 + 5] - xy[k0]
    tang = route.seg / route.L[:, None]
    w = xy[k0] - route.P[:-1]
    tt = np.clip((w * route.seg).sum(1) / route.L ** 2, 0, 1)
    dd = np.linalg.norm(route.P[:-1] + route.seg * tt[:, None] - xy[k0], axis=1)
    dd[(tang @ hd) <= 0] = np.inf
    j = int(dd.argmin())
    xy = xy[k0:]; t_track0 = ref["t"][sub][k0:]
    s_track = [route.cum[j] + tt[j] * route.L[j]]
    for q in xy[1:]:
        s1, d1 = route.project_near(q[None], np.array([s_track[-1]]), 60.0)
        s_track.append(s1[0])
    s_track = np.array(s_track); t_track = t_track0
    # passes coverage (for attendance ratio)
    covered = np.zeros(len(passes), bool)
    for a, b in zip(s_track[:-1], s_track[1:]):
        if 0 <= (b - a) % L < 30:
            i0, i1 = int(a), int(a + (b - a) % L)
            covered[np.arange(i0, i1 + 1) % len(passes)] = True
    passes += covered
    for a in starts:
        e = a
        while e < len(still) and still[e]:
            e += 1
        if t[min(e, len(t) - 1)] - t[a] < 5.0:
            continue
        tm = 0.5 * (t[a] + t[min(e, len(t) - 1)])
        k = np.searchsorted(t_track, tm)
        if k <= 0 or k >= len(t_track) or abs(t_track[k] - tm) > 1.0:
            continue
        stops.append((f.stem, s_track[k]))
print("runs used:", len(seen), "stop episodes:", len(stops))
S = np.array([s for _, s in stops])
# 1-D clustering by density on the circular route
bins = np.zeros(int(L) + 1)
for s in S:
    bins[int(s) % len(bins)] += 1
smooth = np.convolve(np.r_[bins[-5:], bins, bins[:5]], np.ones(7) / 7, mode="same")[5:-5]
stations = []
used = np.zeros(len(S), bool)
for peak in np.argsort(-smooth):
    if smooth[peak] < 1.0:
        break
    dd = (S - peak + L / 2) % L - L / 2
    m = (np.abs(dd) < 12) & ~used
    if m.sum() < 5:
        continue
    c = peak + np.median(dd[m])
    dd2 = (S - c + L / 2) % L - L / 2
    m2 = (np.abs(dd2) < 12) & ~used
    sig = 1.4826 * np.median(np.abs(dd2[m2] - np.median(dd2[m2])))
    npass = passes[int(c) % len(passes)]
    used |= m2
    stations.append(dict(s=float(c % L), sigma=float(max(sig, 0.5)), count=int(m2.sum()), passes=int(npass),
                         attendance=float(m2.sum() / max(npass, 1))))
stations.sort(key=lambda d: d["s"])
# every repeatable stop location (stations and traffic-light stop lines) is a landmark;
# ambiguity between close landmarks is handled at runtime (no snap if two are in the gate)
good = [s for s in stations if s["sigma"] < 3.0 and s["count"] >= 7]
for s in stations:
    flag = "STATION" if s in good else "-"
    print(f"s={s['s']:8.1f} sigma={s['sigma']:5.2f} stops={s['count']:3d} passes={s['passes']:3d} "
          f"attendance={s['attendance']:.2f} {flag}")
final = good
json.dump({"route": "route_closed.json", "stations": final,
           "note": "repeatable stop locations (arc length on route_closed.json), built offline "
                   "from training-run GNSS; used at runtime only when odometry reports standstill"},
          open(ARGS.out, "w"), indent=1)
print("stations kept:", len(final))
