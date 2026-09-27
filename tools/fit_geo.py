"""Офлайн-калибровка преобразования WGS84 (lat/lon/alt) -> map по проверочному bag.

Модель: map_xy = s * R(theta) * ENU(lat, lon | origin) + t ;  map_z = alt + dz
ENU антенны ↔ base_link через рычаг антенны (tf антенн в base_link).
"""
import pickle, sys, json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
bag = pickle.load(open(ROOT / "tools/_cache_check.pkl", "rb"))
LEVER = {"master": np.array([-9.873, 0.0, 3.0]), "rover": np.array([2.563, 0.0, 3.0])}
LAT0, LON0 = 55.80, 37.45
A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)


def ecef(lat, lon, h):
    la, lo = np.radians(lat), np.radians(lon)
    n = A / np.sqrt(1 - E2 * np.sin(la) ** 2)
    return np.stack([(n + h) * np.cos(la) * np.cos(lo), (n + h) * np.cos(la) * np.sin(lo),
                     (n * (1 - E2) + h) * np.sin(la)], -1)


def enu(lat, lon, h, lat0=LAT0, lon0=LON0):
    p = ecef(lat, lon, h) - ecef(np.array(lat0), np.array(lon0), np.array(0.0))
    la, lo = np.radians(lat0), np.radians(lon0)
    R = np.array([[-np.sin(lo), np.cos(lo), 0],
                  [-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)],
                  [np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)]])
    return p @ R.T


ks = bag["/localization/kinematic_state"]; K = ks["v"]; kt = ks["h"]
q = K[:, 3:7]
yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
yaw_u = np.unwrap(yaw)

src, dst, dz, who = [], [], [], []
for ant in ("master", "rover"):
    g = bag[f"/sensing/gnss/{ant}/fix"]
    ok = g["v"][:, 3] >= 0
    t = g["h"][ok]
    # interpolate reference base_link pose at GNSS stamps
    bx = np.interp(t, kt, K[:, 0]); by = np.interp(t, kt, K[:, 1]); bz = np.interp(t, kt, K[:, 2])
    yw = np.interp(t, kt, yaw_u)
    L = LEVER[ant]
    ax = bx + np.cos(yw) * L[0] - np.sin(yw) * L[1]
    ay = by + np.sin(yw) * L[0] + np.cos(yw) * L[1]
    az = bz + L[2]
    e = enu(g["v"][ok, 0], g["v"][ok, 1], g["v"][ok, 2])
    src.append(e); dst.append(np.stack([ax, ay, az], -1)); who += [ant] * len(t)
    dz.append(az - g["v"][ok, 2])
src = np.concatenate(src); dst = np.concatenate(dst); who = np.array(who)


def fit(src, dst):
    # 2D similarity (Umeyama)
    ms, md = src[:, :2].mean(0), dst[:, :2].mean(0)
    S, D = src[:, :2] - ms, dst[:, :2] - md
    U, sig, Vt = np.linalg.svd(D.T @ S / len(S))
    Rm = U @ Vt
    s = sig.sum() / (S ** 2).sum(1).mean()
    t = md - s * Rm @ ms
    return s, Rm, t


mask = np.ones(len(src), bool)
for it in range(3):
    s, Rm, t = fit(src[mask], dst[mask])
    pred = (s * (Rm @ src[:, :2].T)).T + t
    res = np.linalg.norm(pred - dst[:, :2], axis=1)
    mask = res < max(3 * np.median(res[mask]), 0.3)
theta = np.arctan2(Rm[1, 0], Rm[0, 0])
dzv = np.concatenate(dz)
print(f"scale={s:.8f} theta={np.degrees(theta):.5f} deg t={t}")
print(f"inliers {mask.sum()}/{len(mask)}; residual xy: p50={np.median(res[mask]):.3f} p95={np.percentile(res[mask],95):.3f} "
      f"max(all)={res.max():.2f}")
for ant in ("master", "rover"):
    m = who == ant
    print(f"  {ant}: residual p50={np.median(res[m]):.3f} p95={np.percentile(res[m],95):.3f}")
print(f"dz (map_z - alt): median={np.median(dzv):.3f} std={np.std(dzv):.3f}")
out = {"lat0": LAT0, "lon0": LON0, "scale": float(s), "theta_rad": float(theta),
       "tx": float(t[0]), "ty": float(t[1]), "dz": float(np.median(dzv))}
json.dump(out, open(ROOT / "tools/geo_fit.json", "w"), indent=2)
print(out)
