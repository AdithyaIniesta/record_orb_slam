"""
Capture raw-ish IMU from an ArduPilot Cube Orange over USB for Allan variance.
Writes one CSV per IMU: t_fc_us, gx, gy, gz, ax, ay, az  (SI units: rad/s, m/s^2)

Usage:
    python allan_capture.py --port COM7 --hours 6
"""
import argparse, csv, pathlib, sys, time
from pymavlink import mavutil

MRAD_TO_RAD = 1e-3
MG_TO_MS2   = 9.80665e-3

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM7 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hours", type=float, default=6.0)
    ap.add_argument("--rate_hz", type=int, default=200)
    ap.add_argument("--outdir", default=None)
    args = ap.parse_args()

    out = pathlib.Path(args.outdir or
        pathlib.Path(__file__).parent / f"capture_{time.strftime('%Y%m%d_%H%M%S')}")
    out.mkdir(parents=True, exist_ok=True)
    print(f"[allan] output dir: {out}")

    m = mavutil.mavlink_connection(args.port, baud=args.baud)
    print("[allan] waiting for heartbeat...")
    m.wait_heartbeat()
    print(f"[allan] connected: sys {m.target_system} comp {m.target_component}")

    interval_us = int(1_000_000 / args.rate_hz)

    def request_rates():
        # Belt-and-braces: command_long + legacy request_data_stream
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
                       mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
                       mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3):
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval_us, 0, 0, 0, 0, 0)
        m.mav.request_data_stream_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            args.rate_hz, 1)

    request_rates()
    last_rate_req = time.time()

    files   = {k: open(out / f"imu{k}.csv", "w", newline="") for k in (0,1,2)}
    writers = {k: csv.writer(f) for k,f in files.items()}
    for w in writers.values():
        w.writerow(["t_fc_us","gx","gy","gz","ax","ay","az"])

    t_end = time.time() + args.hours*3600
    counts = [0,0,0]
    last = time.time()
    try:
        while time.time() < t_end:
            msg = m.recv_match(blocking=True, timeout=1.0,
                               type=["RAW_IMU","SCALED_IMU2","SCALED_IMU3"])
            if msg is None:
                continue
            t = getattr(msg, "time_usec", None) or msg.time_boot_ms*1000
            if msg.get_type() == "RAW_IMU":
                # RAW_IMU in MAVLink spec: gyro in mrad/s, accel in mg
                writers[0].writerow([t,
                    msg.xgyro*MRAD_TO_RAD, msg.ygyro*MRAD_TO_RAD, msg.zgyro*MRAD_TO_RAD,
                    msg.xacc *MG_TO_MS2,   msg.yacc *MG_TO_MS2,   msg.zacc *MG_TO_MS2])
                counts[0] += 1
            else:
                k = 1 if msg.get_type()=="SCALED_IMU2" else 2
                writers[k].writerow([t,
                    msg.xgyro*MRAD_TO_RAD, msg.ygyro*MRAD_TO_RAD, msg.zgyro*MRAD_TO_RAD,
                    msg.xacc *MG_TO_MS2,   msg.yacc *MG_TO_MS2,   msg.zacc *MG_TO_MS2])
                counts[k] += 1
            if time.time() - last_rate_req > 10:
                request_rates()
                last_rate_req = time.time()
            if time.time()-last > 60:
                remaining = (t_end-time.time())/3600
                print(f"[{time.strftime('%H:%M:%S')}] counts={counts} "
                      f"remaining={remaining:.2f} h")
                last = time.time()
                for f in files.values(): f.flush()
    finally:
        for f in files.values(): f.close()
        print(f"[allan] done. counts={counts}")

if __name__ == "__main__":
    main()
