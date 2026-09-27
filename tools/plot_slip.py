"""Эпизоды рассогласования тележек vs GNSS-скорость."""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
C = ROOT / "tools" / "_cache"
names = sys.argv[1:] or ["30639_3b3d9eb8", "30639_4285f2bc", "30618_2050d396", "30639_d927f360"]
fig, axs = plt.subplots(len(names), 1, figsize=(15, 3.2 * len(names)))
for ax, n in zip(np.atleast_1d(axs), names):
    z = np.load(C / f"{n}.npz")
    ft, fv = z["front_h"], z["front_v"][:, 0] / 3.6
    rt, rv = z["rear_h"], z["rear_v"][:, 0] / 3.6
    t0 = ft[0]
    d = np.abs(fv - np.interp(ft, rt, rv))
    i = np.argmax(d); tc = ft[i]
    w = (ft > tc - 40) & (ft < tc + 40); wr = (rt > tc - 40) & (rt < tc + 40)
    ax.plot(ft[w] - t0, fv[w], ".-", label="front", ms=3)
    ax.plot(rt[wr] - t0, rv[wr], ".-", label="rear", ms=3)
    if "mvel_v" in z:
        gt = z["mvel_h"]; wg = (gt > tc - 40) & (gt < tc + 40)
        ax.plot(gt[wg] - t0, np.hypot(z["mvel_v"][wg, 0], z["mvel_v"][wg, 1]), "k.", label="GNSS", ms=4)
    ct = z["cmd_h"]; wc = (ct > tc - 40) & (ct < tc + 40)
    ax2 = ax.twinx(); ax2.plot(ct[wc] - t0, z["cmd_v"][wc, 0], "g-", alpha=0.4, label="cmd"); ax2.set_ylim(-16, 16)
    ax.set_title(f"{n}: max |front-rear| = {d[i]:.2f} m/s at t={tc - t0:.1f}s"); ax.legend(loc="upper left"); ax.grid()
    # print episode stats
    ep = d > 0.5
    print(n, "samples with |F-R|>0.5 m/s:", ep.sum(), "of", len(d))
fig.tight_layout(); fig.savefig(ROOT / "tools/_slip.png", dpi=90)
