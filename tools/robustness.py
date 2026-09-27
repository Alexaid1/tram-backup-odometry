"""Robustness scenarios on the check bag (reference = /localization/kinematic_state):
fault injection into the real recording, filter vs naive odometry (mean of available bogies,
hold-last on dropout, no outlier handling). Writes tools/_robust.json + docs/figures/robust_*.png."""
import json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from evaluate import evaluate, export_events, inject, load_csv, match, rms, C, OUT, ROOT

RUN = "30618_88aea4d9"
SCEN = [
    ("baseline", []),
    ("no driver command (checker env)", ["INJ:NOCMD=1"]),
    ("pathgraph only (no loops/landmarks)", ["MAPS=pathgraph"]),
    ("landmarks disabled", ["landmarks.enabled=false"]),
    ("rear stuck at 0, 200 s", ["INJ:STUCK_REAR=300,500"]),
    ("rear dropout 100 s", ["INJ:DROP_REAR=300,400"]),
    ("all inputs dropout 3 s at speed", ["INJ:DROP_ALL=486,489"]),
    ("2% wheel outliers (x5)", ["INJ:OUTLIERS=0.02"]),
    ("front spin +30% in traction, 8 s", ["INJ:SLIP_FRONT=470,478,1.3"]),
    ("rear skid -50% in braking, 6 s", ["INJ:SKID_REAR=500,506,0.5"]),
    ("rear stuck + no command", ["INJ:STUCK_REAR=300,500", "INJ:NOCMD=1"]),
]


def naive(events_csv, ref):
    E = np.loadtxt(events_csv, delimiter=",", skiprows=1)
    t = np.arange(E[:, 2].min(), E[:, 2].max(), 0.05)
    out = np.zeros(len(t)); last = {1: 0.0, 2: 0.0}; lt = {1: -1e9, 2: -1e9}
    k = 0
    W = E[(E[:, 0] == 1) | (E[:, 0] == 2)]
    W = W[np.argsort(W[:, 2])]
    for i, tt in enumerate(t):
        while k < len(W) and W[k, 2] <= tt:
            last[int(W[k, 0])] = W[k, 3] / 3.6; lt[int(W[k, 0])] = W[k, 2]; k += 1
        fresh = [last[c] for c in (1, 2) if tt - lt[c] < 0.6]
        out[i] = np.mean(fresh) if fresh else out[i - 1] if i else 0.0
    j, ok = match(t, ref["t"])
    return rms(out[ok] - ref["v"][j[ok]]), float(np.max(np.abs(out[ok] - ref["v"][j[ok]])))


if __name__ == "__main__":
    z = np.load(C / f"{RUN}.npz")
    R = z["ref_v"]; ref = dict(t=z["ref_h"], v=R[:, 7])
    res = []
    for i, (name, sets) in enumerate(SCEN):
        o = evaluate(RUN, sets, plots=(i >= 4), tag=f"_rb{i}")
        nv = naive(OUT / f"{RUN}_rb{i}_events.csv", ref)
        E = load_csv(OUT / f"{RUN}_rb{i}_est.csv")
        o["naive_v_rmse"], o["naive_v_max"] = nv
        o["scenario"] = name
        res.append(o)
        print(f"{name:38s} v_rmse={o['v_rmse']:.3f} (naive {nv[0]:.3f}) v_max={o['v_max']:.2f} (naive {nv[1]:.2f}) "
              f"pos_rmse3d={o['pos_rmse_3d']:.2f} pos_max={o['pos_max_3d']:.1f} "
              f"modes={ {k: round(v, 3) for k, v in o['mode_share'].items()} }", flush=True)
    json.dump(res, open(ROOT / "tools/_robust.json", "w"), indent=1)
