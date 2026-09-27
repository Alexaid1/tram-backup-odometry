"""Summary table of tools/evaluate.py JSON results."""
import json, sys
import numpy as np

R = json.load(open(sys.argv[1]))
R = [r for r in R if "v_rmse" in r]
keys = ["v_rmse", "v_mae", "v_bias", "v_rmse_accel", "v_rmse_brake", "v_rmse_cruise", "v_rmse_stop",
        "pos_rmse_3d", "pos_max_3d", "along_rmse", "along_max", "drift_along_pct", "drift_final_pct",
        "cross_rmse", "proc_us_p99", "pub_rate_hz"]
print(f"runs with reference: {len(R)}")
print(f"{'metric':18s} {'median':>9s} {'mean':>9s} {'p90':>9s} {'max':>9s}")
for k in keys:
    v = np.array([r.get(k, np.nan) for r in R], float); v = v[np.isfinite(v)]
    if len(v):
        print(f"{k:18s} {np.median(v):9.3f} {np.mean(v):9.3f} {np.percentile(v, 90):9.3f} {v.max():9.3f}")
if len(sys.argv) > 2:
    k = sys.argv[2]
    for r in sorted(R, key=lambda r: -r.get(k, 0))[:12]:
        print(f"  {r['run']}: {k}={r.get(k):.2f} pos_rmse={r.get('pos_rmse_3d'):.1f} cross={r.get('cross_rmse'):.2f} "
              f"v_rmse={r['v_rmse']:.3f} modes={r['mode_share']}")
