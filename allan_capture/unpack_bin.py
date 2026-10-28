"""
Unpack an ArduPilot .BIN dataflash log into three CSVs (one per IMU)
matching the schema that allan_fit.py expects:
    t_fc_us, gx, gy, gz, ax, ay, az

Usage:
    python unpack_bin.py path\\to\\000195.BIN
"""
import sys, csv, pathlib, time
from pymavlink import DFReader

def main(bin_path):
    bin_path = pathlib.Path(bin_path)
    outdir = bin_path.parent / (bin_path.stem + "_imu")
    outdir.mkdir(exist_ok=True)
    print(f"[unpack] reading {bin_path}")
    print(f"[unpack] writing to {outdir}")

    files   = {k: open(outdir / f"imu{k}.csv", "w", newline="") for k in (0,1,2)}
    writers = {k: csv.writer(f) for k,f in files.items()}
    for w in writers.values():
        w.writerow(["t_fc_us","gx","gy","gz","ax","ay","az"])

    r = DFReader.DFReader_binary(str(bin_path))
    counts = [0,0,0]
    t0 = time.time()
    while True:
        m = r.recv_match(type="IMU")
        if m is None:
            break
        i = int(m.I)
        if i not in (0,1,2):
            continue
        writers[i].writerow([m.TimeUS,
                             m.GyrX, m.GyrY, m.GyrZ,
                             m.AccX, m.AccY, m.AccZ])
        counts[i] += 1
        if sum(counts) % 500000 == 0:
            print(f"[unpack] counts={counts}  elapsed={time.time()-t0:.1f}s")
    for f in files.values(): f.close()
    print(f"[unpack] done. counts={counts}")
    for k in (0,1,2):
        print(f"  imu{k}.csv : {counts[k]} samples")

if __name__ == "__main__":
    main(sys.argv[1])
