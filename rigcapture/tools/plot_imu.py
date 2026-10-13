#!/usr/bin/env python3
"""Plot the IMU of a RigCapture recording: gyro, accelerometer, |accel|, rotation rate, and timing check.

    python3 plot_imu.py RECORDING_DIR [--out imu.png]

Reads RECORDING_DIR/mav0/imu0/data.csv (EuRoC: t_ns, wx wy wz [rad/s], ax ay az [m/s^2]) and prints a short summary.
Needs numpy and matplotlib.
"""
import argparse, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("rec")
ap.add_argument("--out", default=None)
a = ap.parse_args()
d = np.loadtxt(os.path.join(a.rec, "mav0", "imu0", "data.csv"), delimiter=",", skiprows=1)
t = (d[:, 0] - d[0, 0]) * 1e-9
gyr, acc = d[:, 1:4], d[:, 4:7]
dt = np.diff(t)
anorm = np.linalg.norm(acc, axis=1)
wnorm = np.linalg.norm(gyr, axis=1)
print("%s: %d samples, %.1f s, median rate %.1f Hz, largest gap %.0f ms" % (
    os.path.basename(os.path.abspath(a.rec)), len(t), t[-1], 1.0 / np.median(dt), dt.max() * 1e3))
print("|accel|: median %.2f m/s^2 (gravity 9.81), max %.1f | gyro: max %.2f rad/s (%.0f deg/s), rms %.2f rad/s"
      % (np.median(anorm), anorm.max(), wnorm.max(), np.degrees(wnorm.max()), np.sqrt((wnorm ** 2).mean())))

fig, ax = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
for i, n in enumerate("xyz"):
    ax[0].plot(t, gyr[:, i], lw=0.6, label="w" + n)
    ax[1].plot(t, acc[:, i], lw=0.6, label="a" + n)
ax[0].set_ylabel("gyro [rad/s]"); ax[1].set_ylabel("accel [m/s²]")
ax[2].plot(t, anorm, lw=0.6, color="k"); ax[2].axhline(9.81, color="r", lw=0.8, ls="--")
ax[2].set_ylabel("|accel| [m/s²]\n(dashed = gravity)")
ax[3].plot(t, np.degrees(wnorm), lw=0.6, color="tab:purple"); ax[3].set_ylabel("rotation rate [deg/s]")
ax[3].set_xlabel("time [s]")
for x in ax[:2]: x.legend(loc="upper right", ncol=3, fontsize=8)
for x in ax: x.grid(alpha=0.3)
fig.suptitle("IMU - %s   (%.0f Hz, %.0f s)" % (os.path.basename(os.path.abspath(a.rec)), 1.0 / np.median(dt), t[-1]))
fig.tight_layout()
out = a.out or os.path.join(a.rec, "imu_plot.png")
fig.savefig(out, dpi=110)
print("saved", out)
