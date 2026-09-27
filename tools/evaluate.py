"""Offline validation of the estimator core on bag caches (tools/_cache/*.npz).

For each run: export input events in bag-record order -> run replay.exe (same C++ core
as the ROS node) -> compare with the reference:
  * check bag: /localization/kinematic_state (exactly what the jury checker uses);
  * dataset:   GNSS master fix -> base_link via antenna lever arm, |GNSS vel| for speed.
Metrics mirror the checker (nearest stamp within 0.05 s) + along/cross-track, drift, regimes.

usage: python tools/evaluate.py [run_id ...] [--all] [--plots] [--set name=value ...]
"""
import argparse, json, subprocess, sys, time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
C = ROOT / "tools" / "_cache"
PKG = ROOT / "ros2_ws/src/tram_odometry"
EXE = ROOT / "tools/_bin/replay.exe"
OUT = ROOT / "tools/_eval"
MASTER_X = -9.873


def inject(E, t0, spec):
    """Fault injection on exported events (robustness tests). spec: NAME=a,b[,c]
    STUCK_REAR=t1,t2   rear reads 0            DROP_REAR=t1,t2  no rear messages
    DROP_ALL=t1,t2     no inputs at all        OUTLIERS=p       fraction p of wheel msgs *= 5
    SLIP_FRONT=t1,t2,k front reads k*v         NOCMD=1          no driver command topic
    SKID_REAR=t1,t2,k  rear reads k*v"""
    name, _, arg = spec.partition("=")
    a = [float(x) for x in arg.split(",")] if arg else []
    tr = E[:, 1] - t0
    win = (tr >= a[0]) & (tr <= a[1]) if len(a) >= 2 else None
    if name == "STUCK_REAR":
        E[win & (E[:, 0] == 2), 3] = 0.0
    elif name == "DROP_REAR":
        E = E[~(win & (E[:, 0] == 2))]
    elif name == "DROP_ALL":
        E = E[~(win & (E[:, 0] <= 2))]
    elif name == "OUTLIERS":
        rng = np.random.default_rng(1)
        m = (E[:, 0] >= 1) & (E[:, 0] <= 2) & (rng.random(len(E)) < a[0])
        E[m, 3] *= 5.0
    elif name == "SLIP_FRONT":
        E[win & (E[:, 0] == 1), 3] *= a[2]
    elif name == "SKID_REAR":
        E[win & (E[:, 0] == 2), 3] *= a[2]
    elif name == "NOCMD":
        E = E[E[:, 0] != 0]
    return E


def export_events(z, path, gnss_seconds=None, injections=()):
    rows = []
    def add(kind, key, cols):
        if f"{key}_t" not in z:
            return
        t, h, v = z[f"{key}_t"], z[f"{key}_h"], z[f"{key}_v"]
        vals = np.column_stack([v[:, c] for c in cols] + [np.zeros(len(t))] * (4 - len(cols)))
        rows.append(np.column_stack([np.full(len(t), kind), t, h, vals]))
    add(0, "cmd", [0]); add(1, "front", [0]); add(2, "rear", [0])
    add(3, "mfix", [0, 1, 2, 3]); add(4, "rfix", [0, 1, 2, 3])
    E = np.concatenate(rows)
    t0 = E[E[:, 0] <= 2, 1].min()
    if gnss_seconds is not None:  # simulate "GNSS only in the first seconds"
        E = E[(E[:, 0] <= 2) | (E[:, 1] - t0 <= gnss_seconds)]
    E = E[np.argsort(E[:, 1], kind="stable")]
    for spec in injections:
        E = inject(E, t0, spec)
    with open(path, "w") as f:
        f.write("kind,t_recv,t_stamp,a,b,c,d\n")
        np.savetxt(f, E, fmt=["%d", "%.6f", "%.6f", "%.9f", "%.9f", "%.4f", "%.1f"], delimiter=",")


def load_csv(p):
    with open(p) as f:
        names = f.readline().strip().split(",")
    D = np.loadtxt(p, delimiter=",", skiprows=1, ndmin=2)
    return {n: D[:, i] for i, n in enumerate(names)}


def reference(z):
    """Returns dict(t, x, y, z, v, kind)."""
    if "ref_v" in z:
        R = z["ref_v"]
        return dict(t=z["ref_h"], x=R[:, 0], y=R[:, 1], z=R[:, 2], v=R[:, 7], kind="kinematic_state")
    if "mfix_xy" not in z or len(z["mfix_xy"]) < 100:
        return None
    t = z["mfix_h"]; xy = z["mfix_xy"]; alt = z["mfix_v"][:, 2]
    o = np.argsort(t); t, xy, alt = t[o], xy[o], alt[o]
    h = np.full(len(t), np.nan)
    for i in range(len(t)):
        a = np.searchsorted(t, t[i] - 1.0); b = min(np.searchsorted(t, t[i] + 1.0), len(t) - 1)
        d = xy[b] - xy[a]
        if np.hypot(*d) > 1.0 and (t[b] - t[a]) < 3.0:
            h[i] = np.arctan2(d[1], d[0])
    # hold last known heading while stationary
    last = np.nan
    for i in range(len(h)):
        if np.isnan(h[i]):
            h[i] = last
        else:
            last = h[i]
    first = np.where(~np.isnan(h))[0]
    if not len(first):
        return None
    h[:first[0]] = h[first[0]]
    bx = xy[:, 0] - MASTER_X * np.cos(h); by = xy[:, 1] - MASTER_X * np.sin(h)
    v = np.full(len(t), np.nan)
    if "mvel_v" in z and len(z["mvel_v"]) > 10:
        gv = np.hypot(z["mvel_v"][:, 0], z["mvel_v"][:, 1])
        v = np.interp(t, z["mvel_h"], gv, left=np.nan, right=np.nan)
        # do not interpolate across GNSS outages
        gap = np.interp(t, z["mvel_h"][1:], np.diff(z["mvel_h"]), left=9, right=9)
        v[gap > 0.5] = np.nan
    return dict(t=t, x=bx, y=by, z=alt - 0.14 - 3.0, v=v, kind="gnss_master")


class Route:
    def __init__(self, path):
        d = json.load(open(path, encoding="utf-8"))
        self.P = np.array([[p["x"], p["y"]] for p in d["points"]])
        self.P = np.vstack([self.P, self.P[:1]])
        seg = np.diff(self.P, axis=0); self.seg = seg
        self.L = np.linalg.norm(seg, axis=1); self.cum = np.r_[0, np.cumsum(self.L)]
        self.total = self.cum[-1]

    def project(self, xy, heading=None):
        s = np.empty(len(xy)); d = np.empty(len(xy))
        for i in range(0, len(xy), 500):
            q = xy[i:i + 500]
            w = q[:, None, :] - self.P[None, :-1, :]
            t = np.clip((w * self.seg[None]).sum(-1) / (self.L ** 2)[None], 0, 1)
            pr = self.P[None, :-1, :] + self.seg[None] * t[..., None]
            dd = np.linalg.norm(pr - q[:, None, :], axis=-1)
            j = dd.argmin(1)
            s[i:i + 500] = self.cum[j] + t[np.arange(len(q)), j] * self.L[j]
            d[i:i + 500] = dd[np.arange(len(q)), j]
        return s, d


    def project_near(self, xy, s_hint, window):
        """Projection restricted to route segments within s_hint +- window (same track branch)."""
        s = np.empty(len(xy)); d = np.empty(len(xy))
        n = len(self.L)
        for i, (q, sh) in enumerate(zip(xy, s_hint)):
            k0 = np.searchsorted(self.cum, sh)
            idx = np.arange(k0 - int(window), k0 + int(window)) % n
            w = q - self.P[idx]
            t = np.clip((w * self.seg[idx]).sum(-1) / self.L[idx] ** 2, 0, 1)
            dd = np.linalg.norm(self.P[idx] + self.seg[idx] * t[:, None] - q, axis=1)
            j = dd.argmin()
            s[i] = self.cum[idx[j]] + t[j] * self.L[idx[j]]; d[i] = dd[j]
        return s, d


ROUTE = None


def match(t_est, t_ref, tol=0.05):
    j = np.clip(np.searchsorted(t_ref, t_est), 1, len(t_ref) - 1)
    jj = np.where(np.abs(t_ref[j - 1] - t_est) < np.abs(t_ref[j] - t_est), j - 1, j)
    ok = np.abs(t_ref[jj] - t_est) <= tol
    return jj, ok


def rms(x):
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else np.nan


def evaluate(run, sets=(), plots=False, gnss_seconds=None, tag=""):
    global ROUTE
    z = np.load(C / f"{run}.npz")
    if "cmd_v" not in z and "front_v" not in z:
        return None
    OUT.mkdir(exist_ok=True)
    ev = OUT / f"{run}{tag}_events.csv"; res = OUT / f"{run}{tag}_est.csv"
    if "ref_v" not in z and not any(s.startswith("publish.time_offset") for s in sets):
        sets = list(sets) + ["publish.time_offset=0"]   # GNSS truth has no timing lag
    injections = [s[4:] for s in sets if s.startswith("INJ:")]
    sets = [s for s in sets if not s.startswith("INJ:")]
    export_events(z, ev, gnss_seconds, injections)
    maps = [str(PKG / "maps/route_closed.json"), str(PKG / "maps/stations.json")]
    if any(s.startswith("MAPS=pathgraph") for s in sets):
        maps = [str(p) for p in sorted((PKG / "maps").glob("pathgraph_*.json"))]
    for s_ in sets:
        if s_.startswith("STATIONS="):
            maps = [m for m in maps if "stations" not in m] + ([s_[9:]] if s_[9:] else [])
    sets = [s for s in sets if not s.startswith("MAPS=") and not s.startswith("STATIONS=")]
    t0 = time.time()
    r = subprocess.run([str(EXE), str(PKG / "config/params.yaml"), str(ev), str(res), *maps, *sets],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr); return None
    wall = time.time() - t0
    E = load_csv(res)
    ref = reference(z)
    out = dict(run=run, n_out=len(E["t"]), replay_wall_s=wall,
               proc_us_p50=float(np.percentile(E["proc_us"], 50)),
               proc_us_p99=float(np.percentile(E["proc_us"], 99)),
               proc_us_max=float(E["proc_us"].max()))
    dt_pub = np.diff(E["t"]); out["pub_rate_hz"] = float(1 / np.median(dt_pub))
    out["pub_gap_max_s"] = float(dt_pub.max())
    out["mode_share"] = {k: float(np.mean(E["mode"] == i)) for i, k in
                         enumerate(["normal", "slip", "fault", "dead_reckoning"])}
    if ref is None:
        return out
    out["ref"] = ref["kind"]
    j, ok = match(E["t"], ref["t"])
    ev_ = E["v"][ok] - ref["v"][j[ok]]
    fin = np.isfinite(ev_)
    out["v_rmse"] = rms(ev_); out["v_mae"] = float(np.nanmean(np.abs(ev_))); out["v_bias"] = float(np.nanmean(ev_))
    out["v_max"] = float(np.nanmax(np.abs(ev_))) if fin.any() else np.nan
    out["n_v_pairs"] = int(fin.sum())
    # regimes by reference acceleration
    tr = ref["t"]; vr = ref["v"]
    ar = np.gradient(np.where(np.isfinite(vr), vr, 0), tr)
    arj = ar[j[ok]]; vrj = vr[j[ok]]
    for name, m in (("accel", (arj > 0.2) & (vrj > 0.3)), ("brake", (arj < -0.2) & (vrj > 0.3)),
                    ("cruise", (np.abs(arj) <= 0.2) & (vrj > 0.3)), ("stop", vrj <= 0.3)):
        e = ev_[m]
        out[f"v_rmse_{name}"] = rms(e); out[f"v_bias_{name}"] = float(np.nanmean(e)) if np.isfinite(e).any() else np.nan
    # position (only once initialised)
    okp = ok & (E["valid"] > 0) & (E["init"] > 0)
    jp = j[okp]
    dx = E["x"][okp] - ref["x"][jp]; dy = E["y"][okp] - ref["y"][jp]; dz = E["z"][okp] - ref["z"][jp]
    d3 = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2); d2 = np.hypot(dx, dy)
    out.update(pos_rmse_3d=rms(d3), pos_max_3d=float(d3.max()) if len(d3) else np.nan,
               pos_rmse_x=rms(dx), pos_rmse_y=rms(dy), pos_rmse_z=rms(dz), pos_mean_2d=float(np.mean(d2)) if len(d2) else np.nan,
               n_pos_pairs=int(okp.sum()))
    if ROUTE is None:
        ROUTE = Route(PKG / "maps/route_closed.json")
    if len(jp) > 10:
        sub = np.arange(0, len(jp), 10)
        s_e = E["s_route"][okp][sub] % ROUTE.total
        s_r, d_r = ROUTE.project_near(np.column_stack([ref["x"][jp][sub], ref["y"][jp][sub]]), s_e, 300.0)
        al = (s_e - s_r + ROUTE.total / 2) % ROUTE.total - ROUTE.total / 2
        out.update(along_mean=float(np.mean(np.abs(al))), along_rmse=rms(al), along_max=float(np.max(np.abs(al))),
                   along_final=float(al[-1]), cross_rmse=rms(d_r), cross_max=float(d_r.max()))
        dist = float(np.nansum(np.abs(E["v"][:-1]) * np.diff(E["t"])))
        out["distance_m"] = dist
        out["drift_final_pct"] = float(100 * abs(d2[-1]) / max(dist, 1.0))
        out["drift_along_pct"] = float(100 * abs(al[-1]) / max(dist, 1.0))
    if plots:
        make_plots(run + tag, E, ref, z, j, ok, okp)
    return out


def make_plots(run, E, ref, z, j, ok, okp):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    figdir = ROOT / "docs/figures"; figdir.mkdir(parents=True, exist_ok=True)
    t0 = E["t"][0]
    fig, ax = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    ax[0].plot(ref["t"] - t0, ref["v"], "k-", lw=2, alpha=0.5, label=f"reference ({ref['kind']})")
    ax[0].plot(z["front_h"] - t0, z["front_v"][:, 0] / 3.6, "-", lw=0.7, label="front bogie")
    ax[0].plot(z["rear_h"] - t0, z["rear_v"][:, 0] / 3.6, "-", lw=0.7, label="rear bogie")
    ax[0].plot(E["t"] - t0, E["v"], "r-", lw=1, label="estimate")
    bad = E["mode"] > 0
    ax[0].scatter(E["t"][bad] - t0, E["v"][bad], s=4, c="m", label="slip/fault mode", zorder=5)
    ax[0].set_ylabel("v, m/s"); ax[0].legend(loc="upper right", fontsize=8); ax[0].grid()
    ev_ = np.full(len(E["t"]), np.nan); ev_[ok] = E["v"][ok] - ref["v"][j[ok]]
    ax[1].plot(E["t"] - t0, ev_, "r-", lw=0.6); ax[1].set_ylabel("v error, m/s"); ax[1].grid()
    ax[1].set_ylim(-1, 1)
    ep = np.full(len(E["t"]), np.nan)
    ep[okp] = np.hypot(E["x"][okp] - ref["x"][j[okp]], E["y"][okp] - ref["y"][j[okp]])
    ax[2].plot(E["t"] - t0, ep, "b-"); ax[2].set_ylabel("position error 2D, m"); ax[2].grid()
    ax2 = ax[2].twinx(); ax2.plot(E["t"] - t0, E["mode"], "m-", lw=0.5, alpha=0.5); ax2.set_ylabel("mode")
    ax[2].set_xlabel("t, s")
    fig.tight_layout(); fig.savefig(figdir / f"{run}_velocity.png", dpi=90); plt.close(fig)
    fig, ax = plt.subplots(figsize=(12, 7))
    ax.plot(ref["x"], ref["y"], "k-", lw=3, alpha=0.3, label="reference")
    v = E["valid"] > 0
    ax.plot(E["x"][v], E["y"][v], "r-", lw=1, label="estimate")
    ax.set_aspect("equal"); ax.legend(); ax.grid(); ax.set_title(run)
    fig.savefig(figdir / f"{run}_trajectory.png", dpi=90); plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--plots", action="store_true")
    ap.add_argument("--gnss-seconds", type=float, default=None)
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--tag", default="")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    runs = a.runs or []
    if a.all:
        seen = set()
        for f in sorted(C.glob("*.npz")):
            z = np.load(f)
            if "front_v" not in z or len(z["front_v"]) < 3000:
                continue
            key = (len(z["front_v"]), round(float(z["front_v"][:, 0].sum()), 1))
            if key in seen:
                continue
            seen.add(key); runs.append(f.stem)
    results = []
    for r in runs:
        o = evaluate(r, a.set, a.plots, a.gnss_seconds, a.tag)
        if o is None:
            continue
        results.append(o)
        print(f"{r:16s} ref={o.get('ref','-'):15s} v_rmse={o.get('v_rmse',np.nan):.3f} bias={o.get('v_bias',np.nan):+.3f} "
              f"pos_rmse3d={o.get('pos_rmse_3d',np.nan):7.2f} max={o.get('pos_max_3d',np.nan):7.2f} "
              f"along_rmse={o.get('along_rmse',np.nan):6.2f} drift={o.get('drift_along_pct',np.nan):.3f}% "
              f"cross={o.get('cross_rmse',np.nan):.2f} rate={o['pub_rate_hz']:.0f}Hz "
              f"proc_p99={o['proc_us_p99']:.0f}us modes={ {k: round(v, 3) for k, v in o['mode_share'].items()} }", flush=True)
    if a.json:
        json.dump(results, open(a.json, "w"), indent=1)
