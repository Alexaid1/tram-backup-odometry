"""Report figures (docs/figures): real anomaly episodes, traction model vs data,
processing-time histogram, per-run error distributions."""
import json, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from evaluate import load_csv, ROOT, C, OUT
from calibrate import resample, lag, model

FIG = ROOT / "docs/figures"; FIG.mkdir(parents=True, exist_ok=True)
MODE_NAMES = ["normal", "slip", "fault", "dead-reck."]


def anomaly_panel(ax, run, t_from, t_to, title):
    z = np.load(C / f"{run}.npz"); E = load_csv(OUT / f"{run}_est.csv")
    t0 = z["front_h"][0]
    w = lambda t: (t - t0 >= t_from) & (t - t0 <= t_to)
    m = w(z["front_h"]); ax.plot(z["front_h"][m] - t0, z["front_v"][m, 0] / 3.6, ".-", ms=2, lw=0.8, label="front bogie")
    m = w(z["rear_h"]); ax.plot(z["rear_h"][m] - t0, z["rear_v"][m, 0] / 3.6, ".-", ms=2, lw=0.8, label="rear bogie")
    if "mvel_v" in z:
        m = w(z["mvel_h"]); ax.plot(z["mvel_h"][m] - t0, np.hypot(z["mvel_v"][m, 0], z["mvel_v"][m, 1]), "k.", ms=3, label="GNSS (reference)")
    m = w(E["t"]); ax.plot(E["t"][m] - t0, E["v"][m], "r-", lw=1.5, label="estimate")
    ax2 = ax.twinx()
    ax2.fill_between(E["t"][m] - t0, 0, 1 - E["pgr"][m], color="orange", alpha=0.2, step="post", label="rear P(not grip)")
    ax2.fill_between(E["t"][m] - t0, 0, 1 - E["pgf"][m], color="blue", alpha=0.15, step="post", label="front P(not grip)")
    ax2.set_ylim(0, 3); ax2.set_yticks([0, 0.5, 1]); ax2.set_ylabel("P(slip or fault)")
    ax.set_title(title, fontsize=10); ax.set_ylabel("v, m/s"); ax.grid(alpha=0.3)
    h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=7, loc="upper left")


def fig_anomalies():
    fig, axs = plt.subplots(3, 1, figsize=(13, 11))
    anomaly_panel(axs[0], "30639_3b3d9eb8", 360, 425, "30639_3b3d9eb8: rear sensor stuck at 0 (sensor fault)")
    anomaly_panel(axs[1], "30618_2050d396", 395, 412, "30618_2050d396: rear bogie skid (juz) under braking")
    anomaly_panel(axs[2], "30639_4285f2bc", 0, 70, "30639_4285f2bc: rear topic dropout ~50 s")
    axs[2].set_xlabel("t, s")
    fig.tight_layout(); fig.savefig(FIG / "anomalies_real.png", dpi=100); plt.close(fig)


def fig_model():
    cal = json.load(open(ROOT / "tools/calib.json"))
    p = [cal[k] for k in ("A_t", "gamma_t", "v_base", "p_shape", "A_b", "gamma_b", "c0", "c1", "c2")]
    U, V, A = [], [], []
    for f in sorted(C.glob("*.npz"))[::3]:
        z = np.load(f)
        if "cmd_v" not in z or len(z["front_v"]) < 3000:
            continue
        r = resample(z)
        if r is None:
            continue
        t, u, v, a, ok = r
        uf = lag(u.astype(float), cal["tau"])
        # steady notch: unchanged during the last 2 s (drive response settled)
        from numpy.lib.stride_tricks import sliding_window_view as swv
        W = swv(np.r_[np.full(19, np.nan), u.astype(float)], 20)
        steady = (np.nanmax(W, 1) == np.nanmin(W, 1)) & ~np.isnan(W).any(1)
        m = ok & steady
        U.append(uf[m]); V.append(v[m]); A.append(a[m])
    U, V, A = map(np.concatenate, (U, V, A))
    fig, axs = plt.subplots(1, 2, figsize=(14, 5.5))
    vv = np.linspace(0, 16, 100)
    for k, notch in enumerate((3, 6, 9, 12, 15, -3, -8, -15)):
        col = plt.cm.viridis(k / 8)
        axs[0].plot(vv, model(p, np.full_like(vv, notch), vv), color=col, lw=2, label=f"model u={notch}")
        m = (np.abs(U - notch) < 0.7)
        if m.sum() > 50:
            bins = np.arange(0, 16, 1.0); idx = np.digitize(V[m], bins)
            mean = [np.mean(A[m][idx == i]) if (idx == i).sum() > 20 else np.nan for i in range(1, len(bins))]
            axs[0].plot(bins[:-1] + 0.5, mean, "o", color=col, ms=4)
    axs[0].set_xlabel("v, m/s"); axs[0].set_ylabel("a, m/s^2"); axs[0].grid(alpha=0.3)
    axs[0].set_title("Traction/brake characteristic: model (lines) vs data, notch steady >= 2 s (dots)")
    axs[0].legend(fontsize=7, ncol=2)
    pred = model(p, U, V)
    axs[1].hist2d(pred, A, bins=120, range=[[-1.6, 1.6], [-2, 2]], cmap="Greys", norm=matplotlib.colors.LogNorm())
    axs[1].plot([-1.6, 1.6], [-1.6, 1.6], "r--")
    axs[1].set_xlabel("model acceleration, m/s^2"); axs[1].set_ylabel("measured acceleration, m/s^2")
    axs[1].set_title(f"Model vs measured (test RMS {cal['rms_test']:.3f} m/s^2, std of a {np.std(A):.3f})")
    fig.tight_layout(); fig.savefig(FIG / "traction_model.png", dpi=100); plt.close(fig)


def fig_timing_and_errors():
    E = load_csv(OUT / "30618_88aea4d9_est.csv")
    R = json.load(open(ROOT / "tools/_eval_final.json"))
    R = [r for r in R if "v_rmse" in r]
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.5))
    axs[0].hist(E["proc_us"], bins=np.linspace(0, 30, 61)); axs[0].set_yscale("log")
    axs[0].set_xlabel("processing time per published output, us"); axs[0].set_title(
        f"Core processing time (p50 {np.percentile(E['proc_us'], 50):.1f}, p99 {np.percentile(E['proc_us'], 99):.1f}, max {E['proc_us'].max():.0f} us)")
    v = np.array([r["v_rmse"] for r in R]); pr = np.array([r["pos_rmse_3d"] for r in R])
    axs[1].hist(v, bins=np.linspace(0, 0.3, 31)); axs[1].set_xlabel("velocity RMSE per run, m/s")
    axs[1].set_title(f"{len(R)} runs: median {np.median(v):.3f} m/s")
    axs[2].hist(np.clip(pr, 0, 60), bins=np.linspace(0, 60, 31)); axs[2].set_xlabel("position RMSE 3D per run, m (clipped at 60)")
    axs[2].set_title(f"median {np.median(pr):.2f} m")
    fig.tight_layout(); fig.savefig(FIG / "timing_and_errors.png", dpi=100); plt.close(fig)


if __name__ == "__main__":
    for fn in (fig_anomalies, fig_model, fig_timing_and_errors):
        try:
            fn(); print("ok", fn.__name__)
        except Exception as e:  # noqa
            print("FAILED", fn.__name__, e)
