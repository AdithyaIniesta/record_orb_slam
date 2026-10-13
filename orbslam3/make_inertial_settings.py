#!/usr/bin/env python3
"""Make an ORB-SLAM3 monocular-inertial settings file from a monocular one plus Kalibr results.

    python3 make_inertial_settings.py BASE_MONO.yaml CAMIMU.yaml IMU_NOISE.yaml OUT.yaml

BASE_MONO.yaml  the monocular ORB-SLAM3 settings of the camera (e.g. settings/orbslam_mono_boresight_3000.yaml)
CAMIMU.yaml     Kalibr camera-IMU result in kalibr_noros/results (needs T_imu_cam = camera pose in the IMU frame)
IMU_NOISE.yaml  Kalibr-style IMU noise file (kalibr_noros/inputs/imu_cube_100hz.yaml); same units as ORB-SLAM3 (continuous-time densities,
                ORB-SLAM3 multiplies them by sqrt(frequency) itself)
Prints the time shift to apply to the IMU stamps (run_mono_inertial.sh does that).
"""
import re, sys
import yaml

base, camimu, noise, out = sys.argv[1:5]
text = open(base, encoding="utf-8").read()
text = re.sub(r"\n# ---- IMU \(added.*\Z", "\n", text, flags=re.S)
k = yaml.safe_load(open(camimu))
n = yaml.safe_load(open(noise))
T = k["T_imu_cam"]
rows = ",\n          ".join(", ".join("%.9f" % v for v in r) for r in T)
imu = """
# ---- IMU (added by make_inertial_settings.py from %s and %s) ----
# camera pose in the IMU frame (Kalibr T_imu_cam; ArduPilot FRD IMU)
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [%s]
IMU.NoiseGyro: %.8e
IMU.NoiseAcc: %.8e
IMU.GyroWalk: %.8e
IMU.AccWalk: %.8e
IMU.Frequency: %.1f
IMU.InsertKFsWhenLost: 0
""" % (camimu.replace("\\", "/").split("/")[-1], noise.replace("\\", "/").split("/")[-1], rows,
       n["gyroscope_noise_density"], n["accelerometer_noise_density"],
       n["gyroscope_random_walk"], n["accelerometer_random_walk"], n["update_rate"])
open(out, "w", encoding="utf-8", newline="\n").write(text.rstrip("\n") + "\n" + imu)
print("wrote", out)
print("timeshift_cam_imu_s", k["timeshift_cam_imu_s"], "(t_imu = t_cam + shift; IMU stamps are moved back by this)")
