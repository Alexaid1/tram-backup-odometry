"""2-fold cross-validation of the landmark map: stations built from one half of the runs
(tools/build_stations.py --fold k) are evaluated on the other half, and vice versa.
Also evaluates the same runs without landmarks for comparison."""
import json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from evaluate import evaluate, C, ROOT

runs, seen = [], set()
for f in sorted(C.glob("*.npz")):
    z = np.load(f)
    if "front_v" not in z or len(z["front_v"]) < 3000 or "ref_v" in z:
        continue
    key = (len(z["front_v"]), round(float(z["front_v"][:, 0].sum()), 1))
    if key in seen:
        continue
    seen.add(key)
    runs.append((len(runs) + 1, f.stem))
out = []
for idx, run in runs:
    fold = idx % 2
    other = ROOT / f"tools/_stations_f{1 - fold}.json"   # landmarks from the OTHER half
    r_cv = evaluate(run, [f"STATIONS={other}"], tag="_cv")
    r_no = evaluate(run, ["landmarks.enabled=false"], tag="_nolm")
    if r_cv is None or "pos_rmse_3d" not in r_cv:
        continue
    out.append(dict(run=run, cv=r_cv, nolm=r_no))
    print(f"{run}: pos_rmse no-landmarks={r_no['pos_rmse_3d']:.2f}  cv-landmarks={r_cv['pos_rmse_3d']:.2f}  "
          f"v_rmse {r_no['v_rmse']:.3f}/{r_cv['v_rmse']:.3f}", flush=True)
json.dump(out, open(ROOT / "tools/_cv.json", "w"), indent=1)
for k in ("pos_rmse_3d", "pos_max_3d", "along_rmse", "drift_along_pct", "v_rmse", "v_bias"):
    a = np.array([o["nolm"].get(k, np.nan) for o in out]); b = np.array([o["cv"].get(k, np.nan) for o in out])
    print(f"{k:16s} no-landmarks median {np.nanmedian(a):8.3f} mean {np.nanmean(a):8.3f} | "
          f"CV landmarks median {np.nanmedian(b):8.3f} mean {np.nanmean(b):8.3f}")
