"""Сопоставление систем координат/единиц в проверочном bag."""
import json, sys, pickle
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from inspect_data import read_bag

ROOT = Path(__file__).resolve().parents[1]
BAG = ROOT / "check-code-with-bag/check-code/bags/30618_88aea4d9"
cache = ROOT / "tools/_cache_check.pkl"
if cache.exists():
    bag = pickle.load(open(cache, "rb"))
else:
    bag = read_bag(BAG); pickle.dump(bag, open(cache, "wb"))

ks = bag["/localization/kinematic_state"]
t0 = ks["t"][0]
print("kinematic frame", ks["frame"], "child", ks.get("child"))
K = ks["v"]; kt = ks["h"]
# yaw
qx, qy, qz, qw = K[:, 3], K[:, 4], K[:, 5], K[:, 6]
yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

# bogie velocity vs reference velocity
for name in ("front", "rear"):
    b = bag[f"/vehicle/{name}_bogie_velocity"]
    for lag in (0.0, 0.05, 0.1, 0.2, 0.3):
        vr = np.interp(b["h"] - lag, kt, K[:, 7])
        m = vr > 1.0
        ratio = b["v"][m, 0] / vr[m]
        print(f"{name} lag={lag:.2f}: ratio median={np.median(ratio):.4f} "
              f"(1/3.6={1/3.6:.4f} -> {np.median(ratio)/3.6:.4f}), "
              f"rmse(v/3.6 - vref)={np.sqrt(np.mean((b['v'][:,0]/3.6 - vr)**2)):.3f}")
fr = bag["/vehicle/front_bogie_velocity"]; rr = bag["/vehicle/rear_bogie_velocity"]
vr_r = np.interp(fr["h"], rr["h"], rr["v"][:, 0])
print("front-rear diff (km/h): p50 %.3f p99 %.3f max %.3f" % tuple(np.percentile(np.abs(fr['v'][:,0]-vr_r), [50, 99, 100])))
# quantization
u = np.unique(np.round(np.diff(np.sort(np.unique(fr["v"][:, 0]))), 6))[:10]
print("front velocity value step (quantization):", u)

# pathgraph relation
for f in sorted(ROOT.glob("Pathgraph*/*.json")):
    d = json.load(open(f, encoding="utf-8"))
    P = np.array([[p["x"], p["y"], p["z"]] for p in d["points"]])
    tang = np.array([p["tang"] for p in d["points"]])
    idx = np.arange(0, len(K), 50)
    dists, near = [], []
    for i in idx:
        dd = np.linalg.norm(P[:, :2] - K[i, :2], axis=1)
        j = dd.argmin(); dists.append(dd[j]); near.append(j)
    dists = np.array(dists); near = np.array(near)
    on = dists < 3
    hd = np.angle(np.exp(1j * (yaw[idx] - tang[near])))
    print(f"\n{f.name}: cross-track p50={np.median(dists):.2f} m, share<3m={on.mean():.2f}, "
          f"heading diff(p50 on-path)={np.median(np.abs(hd[on])):.3f} rad")
    print("   z diff p50 on-path:", np.median(K[idx[on], 2] - P[near[on], 2]))
    # track path index evolution over time
    print("   nearest idx (every 50 s):", near[::50].tolist())
    print("   dist (every 50 s):", np.round(dists[::50], 1).tolist())

# GNSS segments
for top in ("/sensing/gnss/master/fix", "/sensing/gnss/rover/fix"):
    g = bag[top]; t = g["t"] - t0
    br = np.where(np.diff(t) > 0.5)[0]
    starts = np.r_[0, br + 1]; ends = np.r_[br, len(t) - 1]
    print(top, "segments:", [(round(t[s], 1), round(t[e], 1)) for s, e in zip(starts, ends)])
    print("  status values:", np.unique(g["v"][:, 3], return_counts=True))

# speed profile summary
print("\nkinematic speed: stopped share (<0.1) %.2f" % np.mean(np.abs(K[:, 7]) < 0.1))
print("start pos", K[0, :3], "yaw", yaw[0], "end pos", K[-1, :3])
print("path length (integrated |v|):", np.sum(np.abs(K[:-1, 7]) * np.diff(kt)))
print("path length (xy):", np.sum(np.linalg.norm(np.diff(K[:, :2], axis=0), axis=1)))
dc = bag["/vehicle/driver_position_cmd"]
vals, cnt = np.unique(dc["v"][:, 0], return_counts=True)
print("driver cmd histogram:", dict(zip(vals.astype(int).tolist(), cnt.tolist())))
