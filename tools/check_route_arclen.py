"""Consistency of the closed route arc length with the reference trajectory (check bag)."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from evaluate import Route, ROOT, PKG

z = np.load(ROOT / "tools/_cache/30618_88aea4d9.npz")
R = z["ref_v"]; t = z["ref_h"]
rt = Route(PKG / "maps/route_closed.json")
sub = slice(0, None, 25)
s, d = rt.project(R[sub, :2]); tt = t[sub]
s = np.unwrap(s / rt.total * 2 * np.pi) / (2 * np.pi) * rt.total
dist = np.r_[0, np.cumsum(R[:-1, 7] * np.diff(t))][sub]
e = (s - s[0]) - (dist - dist[0])
for tq in (0, 30, 50, 70, 100, 150, 200, 250, 400, 600, 800, 1000, 1200, 1250, 1280, 1300):
    i = np.searchsorted(tt - tt[0], tq)
    i = min(i, len(tt) - 1)
    print(f"t={tq:5d}  s={s[i]:8.1f}  cross={d[i]:5.2f}  (s-s0)-(dist-dist0)={e[i]:7.2f}")
