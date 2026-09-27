"""Офлайн-калибровка модели тяги/торможения и сопротивления по датасету.

Модель (удельная, на единицу массы, м/с^2):
  u_f  = фильтр 1-го порядка (tau) от ручки u ∈ [-15..15]  (задержка привода)
  a_tr = A_t * (u_f/15)^g_t * 1/(1+(v/v_b)^p)^(1/p)          u_f > 0
  a_br = -A_b * (|u_f|/15)^g_b * tanh(v/v_eps)               u_f < 0
  a    = a_tr + a_br - (c0 + c1 v + c2 v^2) * [v>0]
Параметры находятся нелинейным МНК по ускорению, полученному дифференцированием
согласованной скорости тележек (|front-rear| < 0.1 м/с).
"""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

ROOT = Path(__file__).resolve().parents[1]
C = ROOT / "tools" / "_cache"
DT = 0.1
UMAX = 15.0


def resample(z):
    ft, fv = z["front_h"], z["front_v"][:, 0] / 3.6
    rt, rv = z["rear_h"], z["rear_v"][:, 0] / 3.6
    ct, cv = z["cmd_h"], z["cmd_v"][:, 0]
    o = np.argsort(ft); ft, fv = ft[o], fv[o]
    o = np.argsort(rt); rt, rv = rt[o], rv[o]
    o = np.argsort(ct); ct, cv = ct[o], cv[o]
    t = np.arange(max(ft[0], rt[0], ct[0]) + 1, min(ft[-1], rt[-1], ct[-1]) - 1, DT)
    if len(t) < 200:
        return None
    f = np.interp(t, ft, fv); r = np.interp(t, rt, rv)
    # zero-order hold for cmd
    idx = np.clip(np.searchsorted(ct, t, side="right") - 1, 0, len(cv) - 1)
    u = cv[idx]
    # validity: sensors fresh & agree
    gf = np.interp(t, ft[1:], np.diff(ft)); gr = np.interp(t, rt[1:], np.diff(rt))
    ok = (np.abs(f - r) < 0.1) & (gf < 0.35) & (gr < 0.35)
    v = 0.5 * (f + r)
    vs = savgol_filter(v, 11, 2)
    a = savgol_filter(v, 11, 2, deriv=1, delta=DT)
    return t, u, vs, a, ok


def lag(u, tau):
    k = DT / (tau + DT)
    y = np.empty_like(u); s = 0.0
    for i, x in enumerate(u):
        s += k * (x - s); y[i] = s
    return y


def model(p, uf, v):
    At, gt, vb, pp, Ab, gb, c0, c1, c2 = p
    un = uf / UMAX
    atr = At * np.clip(un, 0, None) ** gt / (1 + (v / vb) ** pp) ** (1 / pp)
    abr = -Ab * np.clip(-un, 0, None) ** gb * np.tanh(v / 0.3)
    res = (c0 + c1 * v + c2 * v * v) * np.tanh(v / 0.1)
    return atr + abr - res


if __name__ == "__main__":
    files = sorted(C.glob("*.npz"))
    seen = set(); data = []
    for f in files:
        z = np.load(f)
        if "cmd_v" not in z or len(z["front_v"]) < 1000:
            continue
        key = (len(z["front_v"]), round(float(z["front_v"][:, 0].sum()), 1))
        if key in seen:  # дубликаты прогонов
            continue
        seen.add(key)
        r = resample(z)
        if r is not None:
            data.append((f.stem, r))
    print("unique runs:", len(data))
    # train/test split by run (every 4th run -> test)
    train = [d for i, d in enumerate(data) if i % 4 != 0]
    test = [d for i, d in enumerate(data) if i % 4 == 0]

    def stack(ds, tau):
        U, V, A = [], [], []
        for _, (t, u, v, a, ok) in ds:
            uf = lag(u.astype(float), tau)
            m = ok & (v > 0.05) | (ok & (np.abs(u) > 0) & (v > 0.0))
            m &= ok
            U.append(uf[m]); V.append(v[m]); A.append(a[m])
        return np.concatenate(U), np.concatenate(V), np.concatenate(A)

    p0 = [1.2, 1.0, 8.0, 2.0, 1.3, 1.0, 0.03, 0.0, 0.0005]
    lb = [0.1, 0.3, 1.0, 1.0, 0.1, 0.3, -0.2, -0.05, 0.0]
    ub = [3.0, 3.0, 30.0, 8.0, 3.0, 3.0, 0.3, 0.05, 0.01]
    best = None
    for tau in (0.0, 0.3, 0.6, 0.9, 1.2, 1.6, 2.0):
        U, V, A = stack(train, tau)
        sel = np.random.default_rng(0).choice(len(U), min(len(U), 300000), replace=False)
        r = least_squares(lambda p: model(p, U[sel], V[sel]) - A[sel], p0, bounds=(lb, ub), loss="soft_l1", f_scale=0.2)
        rms = np.sqrt(np.mean((model(r.x, U, V) - A) ** 2))
        print(f"tau={tau:.1f}: rms={rms:.4f}  p={np.round(r.x, 4).tolist()}")
        if best is None or rms < best[0]:
            best = (rms, tau, r.x)
    rms, tau, p = best
    Ut, Vt, At = stack(test, tau)
    rms_test = np.sqrt(np.mean((model(p, Ut, Vt) - At) ** 2))
    print(f"BEST tau={tau} train rms={rms:.4f} test rms={rms_test:.4f} (std of a: {At.std():.4f})")
    # open-loop simulation error on test runs: integrate model from true v
    names = ["A_t", "gamma_t", "v_base", "p_shape", "A_b", "gamma_b", "c0", "c1", "c2"]
    out = dict(zip(names, map(float, p))); out["tau"] = tau
    out["rms_train"] = float(rms); out["rms_test"] = float(rms_test)
    # scale factor per wheel vs GNSS speed
    ks = []
    for f in files:
        z = np.load(f)
        if "mvel_v" not in z or len(z["mvel_v"]) < 1000:
            continue
        gt = z["mvel_h"]; gs = np.hypot(z["mvel_v"][:, 0], z["mvel_v"][:, 1])
        fv = np.interp(gt, z["front_h"], z["front_v"][:, 0] / 3.6)
        rv = np.interp(gt, z["rear_h"], z["rear_v"][:, 0] / 3.6)
        m = (gs > 3) & (np.abs(fv - rv) < 0.1)
        if m.sum() > 500:
            ks.append((f.stem, np.median(gs[m] / fv[m]), np.median(gs[m] / rv[m])))
    ks_arr = np.array([k[1:] for k in ks])
    print("GNSS/odo scale front: median %.5f p10 %.5f p90 %.5f; rear median %.5f" % (
        np.median(ks_arr[:, 0]), np.percentile(ks_arr[:, 0], 10), np.percentile(ks_arr[:, 0], 90), np.median(ks_arr[:, 1])))
    out["odo_scale_front"] = float(np.median(ks_arr[:, 0]))
    out["odo_scale_rear"] = float(np.median(ks_arr[:, 1]))
    # physical plausibility stats of acceleration (for slip thresholds)
    Ua, Va, Aa = stack(data, 0.0)
    out["accel_p0_1"] = float(np.percentile(Aa, 0.1)); out["accel_p99_9"] = float(np.percentile(Aa, 99.9))
    print("accel p0.1 / p99.9:", out["accel_p0_1"], out["accel_p99_9"])
    json.dump(out, open(ROOT / "tools/calib.json", "w"), indent=2)
    print(json.dumps(out, indent=2))
