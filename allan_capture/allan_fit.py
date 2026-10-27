"""
Fit Allan deviation on captured IMU CSVs and print Kalibr yaml numbers.

Usage:
    python allan_fit.py capture_YYYYmmdd_HHMMSS/imu0.csv
"""
import sys, numpy as np, pandas as pd, allantools, matplotlib.pyplot as plt, pathlib

def fit_axis(t_s, x, label):
    fs = 1.0 / np.median(np.diff(t_s))
    taus = np.logspace(np.log10(2/fs), np.log10((t_s[-1]-t_s[0])/5), 60)
    taus_out, adev, _, _ = allantools.oadev(x, rate=fs, data_type="freq", taus=taus)
    # noise density at tau=1s  (ADev * sqrt(tau))
    i1 = np.argmin(np.abs(taus_out-1.0))
    nd = adev[i1]           # units: [x]/sqrt(Hz) * ... see below
    bi = adev.min()*0.664   # bias instability
    return taus_out, adev, nd, bi, fs

def main(csv_path):
    df = pd.read_csv(csv_path)
    t_s = (df["t_fc_us"].values - df["t_fc_us"].values[0]) * 1e-6
    print(f"loaded {len(df)} samples, duration {t_s[-1]/3600:.2f} h")

    results = {}
    fig, axs = plt.subplots(1,2, figsize=(12,5))
    for col, ax in [("gx","gyro"), ("ax","accel")]:
        pass

    for axis_set, title, idx in [(["gx","gy","gz"], "gyro", 0),
                                 (["ax","ay","az"], "accel", 1)]:
        nds, bis = [], []
        for col in axis_set:
            taus, adev, nd, bi, fs = fit_axis(t_s, df[col].values, col)
            axs[idx].loglog(taus, adev, label=col)
            nds.append(nd); bis.append(bi)
        axs[idx].set_title(f"{title}  (fs={fs:.1f} Hz)")
        axs[idx].set_xlabel("tau [s]"); axs[idx].set_ylabel("Allan dev")
        axs[idx].grid(True, which="both"); axs[idx].legend()
        results[title] = (np.mean(nds), np.mean(bis))

    out_png = pathlib.Path(csv_path).with_suffix(".png")
    plt.tight_layout(); plt.savefig(out_png, dpi=120)
    print(f"[fit] plot saved -> {out_png}")

    gnd, gbi = results["gyro"]
    and_, abi = results["accel"]
    print("\n=== Kalibr IMU yaml (SI units) ===")
    print(f"gyroscope_noise_density:      {gnd:.6e}   # rad/s/sqrt(Hz)")
    print(f"gyroscope_random_walk:        {gbi:.6e}   # rad/s^2/sqrt(Hz) (approx from bias instab.)")
    print(f"accelerometer_noise_density:  {and_:.6e}   # m/s^2/sqrt(Hz)")
    print(f"accelerometer_random_walk:    {abi:.6e}   # m/s^3/sqrt(Hz) (approx from bias instab.)")
    print("# NOTE: multiply densities by 5-10x when feeding a VIO filter.")

if __name__ == "__main__":
    main(sys.argv[1])
