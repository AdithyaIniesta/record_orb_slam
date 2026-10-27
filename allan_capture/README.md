# Allan Capture — Cube Orange over USB

One-click Windows tool to record 6 h of IMU data from an ArduPilot Cube Orange
over USB and fit Allan deviation into Kalibr-ready yaml numbers.

## Quick start

1. **Plug** the Cube Orange into the PC over USB. Note the COM port in Device Manager.
2. **Set ArduPilot params** once in Mission Planner (write + reboot):

   | Param | Value |
   |---|---|
   | `INS_GYRO_FILTER`  | 256 |
   | `INS_HNTCH_ENABLE` | 0 |
   | `INS_LOG_BAT_MASK` | 7 |
   | `INS_LOG_BAT_OPT`  | 4 |
   | `LOG_DISARMED`     | 1 |
   | `SR0_RAW_SENS`     | 200 |

3. **Mount** the Cube on a heavy pad on a floor nobody walks on. Warm-soak **30 min**.
4. Double-click **`install.bat`** (needs Python 3.10+ on PATH).
5. Edit `run.bat` → set `PORT=COMx` and `HOURS=6`.
6. Double-click **`run.bat`**. Walk away for 6.5 h.
7. Also pull the Cube's SD `.BIN` log via Mission Planner → DataFlash Logs
   (gold copy at ~1 kHz; the CSV is the backup).
8. Drag `capture_*/imu0.csv` onto **`analyse.bat`** → prints the four Kalibr yaml
   numbers and saves an Allan-deviation plot.

## Output

```
capture_YYYYmmdd_HHMMSS/
  imu0.csv   imu1.csv   imu2.csv       # one per Cube IMU (ICM-42688 x2, ICM-20649)
  imu0.png                             # Allan plot after analyse.bat
```

## Notes

- Raw values from MAVLink are already SI-scaled in the script
  (rad/s and m/s², via mg/mrad conversions where needed).
- VIO estimators typically want the Allan noise densities multiplied by 5–10×.
- Don't skip the thermal soak — the +1 ramp at long τ is almost always "started
  too cold".
