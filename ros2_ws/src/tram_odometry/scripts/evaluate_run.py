#!/usr/bin/env python3
"""Metrics of a recorded run (output of result_recorder.py) against the reference.

Reference: /localization/kinematic_state if it was in the bag (what the jury checker uses),
otherwise GNSS master fix -> base_link (antenna lever arm) and |GNSS velocity|.
Only numpy is required; matplotlib (optional) produces plots.

usage: evaluate_run.py <out_dir> [route_closed.json]
Prints a metrics table and writes <out_dir>/metrics.json (+ PNG plots if matplotlib exists).
"""
import json
import math
import os
import sys

import numpy as np

MASTER_X = -9.873
MAP_OFFSET = (300000.0, 6100000.0, -0.14)


def utm(lat, lon, zone=37):
    """WGS84 -> UTM (Krueger series), vectorised; matches the C++ implementation."""
    a, f, k0 = 6378137.0, 1 / 298.257223563, 0.9996
    n = f / (2 - f)
    A = a / (1 + n) * (1 + n ** 2 / 4 + n ** 4 / 64 + n ** 6 / 256)
    al = [n / 2 - 2 * n ** 2 / 3 + 5 * n ** 3 / 16 + 41 * n ** 4 / 180 - 127 * n ** 5 / 288 + 7891 * n ** 6 / 37800,
          13 * n ** 2 / 48 - 3 * n ** 3 / 5 + 557 * n ** 4 / 1440 + 281 * n ** 5 / 630 - 1983433 * n ** 6 / 1935360,
          61 * n ** 3 / 240 - 103 * n ** 4 / 140 + 15061 * n ** 5 / 26880 + 167603 * n ** 6 / 181440,
          49561 * n ** 4 / 161280 - 179 * n ** 5 / 168 + 6601661 * n ** 6 / 7257600,
          34729 * n ** 5 / 80640 - 3418889 * n ** 6 / 1995840, 212378941 * n ** 6 / 319334400]
    lat = np.radians(lat); dl = np.radians(lon) - math.radians(zone * 6 - 183)
    e = math.sqrt(f * (2 - f))
    t = np.sinh(np.arctanh(np.sin(lat)) - e * np.arctanh(e * np.sin(lat)))
    xi_p = np.arctan2(t, np.cos(dl)); eta_p = np.arctanh(np.sin(dl) / np.sqrt(1 + t * t))
    xi, eta = xi_p.copy(), eta_p.copy()
    for j in range(1, 7):
        xi += al[j - 1] * np.sin(2 * j * xi_p) * np.cosh(2 * j * eta_p)
        eta += al[j - 1] * np.cos(2 * j * xi_p) * np.sinh(2 * j * eta_p)
    return 500000 + k0 * A * eta, k0 * A * xi


def read_csv(path, str_cols=()):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        head = f.readline().strip().split(",")
        rows = [l.rstrip("\n").split(",") for l in f if l.strip()]
    if not rows:
        return None
    out = {}
    for i, h in enumerate(head):
        col = [r[i] if i < len(r) else "" for r in rows]
        out[h] = np.array(col) if h in str_cols else np.array([float(c) if c else np.nan for c in col])
    return out


def reference(d):
    ref = read_csv(os.path.join(d, "reference.csv"))
    if ref is not None and len(ref["stamp"]) > 10:
        o = np.argsort(ref["stamp"])
        return {k: v[o] for k, v in ref.items()} | {"kind": "kinematic_state"}
    g = read_csv(os.path.join(d, "gnss.csv"), str_cols=("topic",))
    if g is None:
        return None
    m = g["topic"] == "master_fix"
    if m.sum() < 20:
        return None
    t = g["stamp"][m]; o = np.argsort(t); t = t[o]
    e, n = utm(g["lat"][m][o], g["lon"][m][o])
    x = e - MAP_OFFSET[0]; y = n - MAP_OFFSET[1]; z = g["alt"][m][o] + MAP_OFFSET[2] - 3.0
    h = np.full(len(t), np.nan)
    for i in range(len(t)):
        a = np.searchsorted(t, t[i] - 1.0); b = min(np.searchsorted(t, t[i] + 1.0), len(t) - 1)
        if math.hypot(x[b] - x[a], y[b] - y[a]) > 1.0 and t[b] - t[a] < 3.0:
            h[i] = math.atan2(y[b] - y[a], x[b] - x[a])
    last = np.nan
    for i in range(len(h)):
        h[i] = last if np.isnan(h[i]) else h[i]; last = h[i]
    ok = ~np.isnan(h)
    if not ok.any():
        return None
    h[~ok] = h[ok][0]
    v = np.full(len(t), np.nan)
    mv = g["topic"] == "master_vel"
    if mv.sum() > 10:
        tv = g["stamp"][mv]; ov = np.argsort(tv)
        v = np.interp(t, tv[ov], np.hypot(g["vx"][mv][ov], g["vy"][mv][ov]), left=np.nan, right=np.nan)
    return {"stamp": t, "x": x - MASTER_X * np.cos(h), "y": y - MASTER_X * np.sin(h), "z": z, "v": v,
            "kind": "gnss_master (lever-arm corrected)"}


def match(t_est, t_ref, tol=0.05):
    j = np.clip(np.searchsorted(t_ref, t_est), 1, len(t_ref) - 1)
    jj = np.where(np.abs(t_ref[j - 1] - t_est) < np.abs(t_ref[j] - t_est), j - 1, j)
    return jj, np.abs(t_ref[jj] - t_est) <= tol


def rms(x):
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float("nan")


def project_tracked(route_xy, cum, q, yaw0, s_hint=None, window=300):
    """Branch-consistent projection: the first point uses the segment aligned with yaw0,
    next points stay within +-window m of the previous (or hinted) arc position; the closed
    route contains both directions 3.5 m apart, so a plain nearest projection can jump."""
    seg = np.diff(route_xy, axis=0); L = np.linalg.norm(seg, axis=1); n = len(L)
    tang = seg / L[:, None]
    out_s = np.empty(len(q)); out_d = np.empty(len(q))
    prev = None
    for i, p in enumerate(q):
        if s_hint is not None:
            k0 = np.searchsorted(cum, s_hint[i] % cum[-1]); idx = np.arange(k0 - window, k0 + window) % n
        elif prev is None:
            idx = np.flatnonzero(tang @ np.array([math.cos(yaw0), math.sin(yaw0)]) > 0.5)
        else:
            k0 = np.searchsorted(cum, prev); idx = np.arange(k0 - window, k0 + window) % n
        w = p - route_xy[idx]
        t = np.clip((w * seg[idx]).sum(1) / L[idx] ** 2, 0, 1)
        dd = np.linalg.norm(route_xy[idx] + seg[idx] * t[:, None] - p, axis=1)
        j = dd.argmin()
        out_s[i] = cum[idx[j]] + t[j] * L[idx[j]]; out_d[i] = dd[j]; prev = out_s[i]
    return out_s, out_d


def project(route_xy, cum, q):
    seg = np.diff(route_xy, axis=0); L2 = (seg ** 2).sum(1); L = np.sqrt(L2)
    s = np.empty(len(q)); d = np.empty(len(q))
    for i in range(0, len(q), 400):
        b = q[i:i + 400]
        w = b[:, None, :] - route_xy[None, :-1, :]
        tt = np.clip((w * seg[None]).sum(-1) / L2[None], 0, 1)
        dd = np.linalg.norm(route_xy[None, :-1, :] + seg[None] * tt[..., None] - b[:, None, :], axis=-1)
        k = dd.argmin(1); r = np.arange(len(b))
        s[i:i + 400] = cum[k] + tt[r, k] * L[k]; d[i:i + 400] = dd[r, k]
    return s, d


def main():
    d = sys.argv[1]
    route_file = sys.argv[2] if len(sys.argv) > 2 else None
    if route_file is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            route_file = os.path.join(get_package_share_directory("tram_odometry"), "maps", "route_closed.json")
        except Exception:  # noqa: BLE001
            route_file = None
    vel = read_csv(os.path.join(d, "result_velocity.csv"))
    pos = read_csv(os.path.join(d, "result_position.csv"), str_cols=("frame",))
    inp = read_csv(os.path.join(d, "inputs.csv"), str_cols=("topic",))
    res = read_csv(os.path.join(d, "resources.csv"))
    M = {}
    if vel is None:
        print("no /result/velocity recorded"); return
    # ---- real-time metrics
    M["published_velocity"] = int(len(vel["stamp"]))
    M["published_position"] = int(len(pos["stamp"])) if pos is not None else 0
    dts = np.diff(vel["recv_wall"]); M["pub_rate_hz_wall"] = float(len(dts) / max(dts.sum(), 1e-9))
    M["pub_gap_wall_max_s"] = float(dts.max()) if len(dts) else float("nan")
    if inp is not None:
        in_by_stamp = {}
        for s, w in zip(inp["stamp"], inp["recv_wall"]):
            in_by_stamp.setdefault(round(s, 6), w)
        lat = np.array([w - in_by_stamp[round(s, 6)] for s, w in zip(vel["stamp"], vel["recv_wall"])
                        if round(s, 6) in in_by_stamp]) * 1000.0
        if len(lat):
            M.update(latency_ms_p50=float(np.percentile(lat, 50)), latency_ms_p95=float(np.percentile(lat, 95)),
                     latency_ms_max=float(lat.max()), latency_samples=int(len(lat)))
            np.savetxt(os.path.join(d, "latency_ms.txt"), lat, fmt="%.3f")
    if res is not None and len(res["wall"]):
        M.update(cpu_percent_mean=float(np.mean(res["cpu_percent"])), cpu_percent_max=float(np.max(res["cpu_percent"])),
                 rss_mb_max=float(np.max(res["rss_mb"])),
                 rss_mb_growth=float(res["rss_mb"][-1] - res["rss_mb"][min(5, len(res["rss_mb"]) - 1)]))
    # ---- accuracy
    ref = reference(d)
    if ref is not None:
        M["reference"] = ref["kind"]
        tr = ref["stamp"]
        j, ok = match(vel["stamp"], tr)
        e = vel["velocity"][ok] - ref["v"][j[ok]]
        M.update(v_rmse=rms(e), v_mae=float(np.nanmean(np.abs(e))), v_bias=float(np.nanmean(e)),
                 v_max=float(np.nanmax(np.abs(e))), v_pairs=int(np.isfinite(e).sum()))
        ar = np.gradient(np.nan_to_num(ref["v"]), tr)[j[ok]]; vr = ref["v"][j[ok]]
        for name, m in (("accel", (ar > 0.2) & (vr > 0.3)), ("brake", (ar < -0.2) & (vr > 0.3)),
                        ("cruise", (np.abs(ar) <= 0.2) & (vr > 0.3)), ("stop", vr <= 0.3)):
            M[f"v_rmse_{name}"] = rms(e[m]); M[f"v_bias_{name}"] = float(np.nanmean(e[m])) if m.any() else float("nan")
        if pos is not None and len(pos["stamp"]):
            jp, okp = match(pos["stamp"], tr)
            dx = pos["x"][okp] - ref["x"][jp[okp]]; dy = pos["y"][okp] - ref["y"][jp[okp]]
            dz = pos["z"][okp] - ref["z"][jp[okp]]
            d3 = np.sqrt(dx * dx + dy * dy + dz * dz)
            M.update(pos_rmse_x=rms(dx), pos_rmse_y=rms(dy), pos_rmse_z=rms(dz), pos_rmse_3d=rms(d3),
                     pos_max_3d=float(d3.max()) if len(d3) else float("nan"), pos_pairs=int(okp.sum()))
            dist = float(np.nansum(vel["velocity"][:-1] * np.clip(np.diff(vel["stamp"]), 0, 1)))
            M["distance_m"] = dist
            if len(d3):
                M["drift_final_pct"] = 100.0 * float(np.hypot(dx[-1], dy[-1])) / max(dist, 1.0)
            if route_file and os.path.exists(route_file) and okp.sum() > 20:
                rj = json.load(open(route_file, encoding="utf-8"))
                P = np.array([[p["x"], p["y"]] for p in rj["points"]]); P = np.vstack([P, P[:1]])
                cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
                sub = np.arange(0, okp.sum(), 10)
                se, _ = project_tracked(P, cum, np.column_stack([pos["x"][okp][sub], pos["y"][okp][sub]]),
                                        float(pos["yaw"][okp][sub][0]))
                sr, cr = project_tracked(P, cum, np.column_stack([ref["x"][jp[okp]][sub], ref["y"][jp[okp]][sub]]),
                                         0.0, s_hint=se)
                al = (se - sr + cum[-1] / 2) % cum[-1] - cum[-1] / 2
                M.update(along_mean=float(np.mean(np.abs(al))), along_rmse=rms(al), along_max=float(np.max(np.abs(al))),
                         cross_rmse_ref_to_route=rms(cr), drift_along_final_pct=100.0 * abs(al[-1]) / max(dist, 1.0))
    json.dump(M, open(os.path.join(d, "metrics.json"), "w"), indent=2)
    print("=" * 60)
    for k, v in M.items():
        print(f"{k:28s} {v:.4f}" if isinstance(v, float) else f"{k:28s} {v}")
    print("=" * 60)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(14, 8))
        t0 = vel["stamp"][0]
        if ref is not None:
            ax[0].plot(ref["stamp"] - t0, ref["v"], "k-", alpha=0.5, lw=2, label="reference")
        if inp is not None:
            for tp in ("front", "rear"):
                m = inp["topic"] == tp
                ax[0].plot(inp["stamp"][m] - t0, inp["value"][m] / 3.6, lw=0.7, label=f"{tp} bogie")
        ax[0].plot(vel["stamp"] - t0, vel["velocity"], "r-", lw=1, label="/result/velocity")
        ax[0].legend(); ax[0].grid(); ax[0].set_ylabel("m/s")
        if "latency_ms_p50" in M:
            ax[1].hist(np.loadtxt(os.path.join(d, "latency_ms.txt")), bins=100)
            ax[1].set_xlabel("input -> /result/velocity latency, ms"); ax[1].set_yscale("log")
        fig.tight_layout(); fig.savefig(os.path.join(d, "run_summary.png"), dpi=90)
        if ref is not None and pos is not None:
            fig, ax = plt.subplots(figsize=(12, 7))
            ax.plot(ref["x"], ref["y"], "k-", lw=3, alpha=0.3, label="reference")
            ax.plot(pos["x"], pos["y"], "r-", lw=1, label="/result/position")
            ax.set_aspect("equal"); ax.legend(); ax.grid()
            fig.savefig(os.path.join(d, "trajectory.png"), dpi=90)
        print(f"plots saved to {d}")
    except ImportError:
        print("matplotlib not installed: plots skipped")


if __name__ == "__main__":
    main()
