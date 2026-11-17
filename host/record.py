"""Record one scan of the ESP32 potentiostat into a file named as data/potentiostat/PROTOCOL.md says.

From the project root:
    python -I firmware/host/record.py run                       locked CV, next free name of today
    python -I firmware/host/record.py run --name 20261120_007   the name the run sheet assigned
    python -I firmware/host/record.py dummy 10000               dummy cell of 10 kohm
    python -I firmware/host/record.py info                      configuration and calibration only
Options: --port COM5 (default: the only USB serial port found), --out DIR (default
data/potentiostat/scans).

The file holds exactly what the firmware sends from "# begin scan" to "# end scan", after one
comment line written here (date, time, port). An existing file is never overwritten. A scan
interrupted with Ctrl+C is stopped on the instrument and saved as it is: every scan is kept
(PROTOCOL.md, rule 3). Next free name: the highest number of the day among the files present
and the names already in manifest.csv, plus one. Needs pyserial.
"""
import argparse
import csv
import datetime as dt
import re
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is missing: python -m pip install pyserial")

ROOT = Path(__file__).resolve().parents[2]
SCANS = ROOT / "data" / "potentiostat" / "scans"
MANIFEST = ROOT / "data" / "potentiostat" / "manifest.csv"
BAUD = 115200
NAME = re.compile(r"^\d{8}_\d{3}$")
SILENCE_LIMIT_S = 30.0  # longest gap allowed between two lines (the quiet time is 5 s)


def next_free_name(out: Path, day: str) -> str:
    used = {p.stem for p in out.glob(f"{day}_*.csv")}
    if MANIFEST.exists():
        with MANIFEST.open(newline="", encoding="utf-8") as f:
            used |= {Path(r.get("file") or "").stem for r in csv.DictReader(f)}
    nums = [int(s[9:]) for s in used if NAME.match(s) and s.startswith(day + "_")]
    return f"{day}_{max(nums, default=0) + 1:03d}"


def find_port(given):
    if given:
        return given
    ports = [p for p in list_ports.comports() if p.vid is not None]  # USB serial adapters only
    if len(ports) == 1:
        return ports[0].device
    found = ", ".join(f"{p.device} ({p.description})" for p in ports) or "none"
    sys.exit(f"give the port with --port; USB serial ports found: {found}")


def open_port(port):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, BAUD, 0.5
    s.dtr = False  # keep the board's auto-reset lines released
    s.rts = False
    s.open()
    time.sleep(1.5)  # if the board reset anyway, let it boot
    s.reset_input_buffer()
    return s


def readline(s):
    raw = s.readline()
    return raw.decode("ascii", "replace").rstrip("\r\n") if raw else None


def show_info(s):
    s.write(b"\ninfo\n")
    last = time.monotonic()
    while time.monotonic() - last < 1.5:
        line = readline(s)
        if line is not None:
            last = time.monotonic()
            if line.startswith("#"):
                print(line)


def record(s, command):
    """Lines from '# begin scan' to '# end scan', and whether the scan ended normally."""
    s.write(b"\n" + command.encode("ascii") + b"\n")
    lines, started, points, last = [], False, 0, time.monotonic()
    try:
        while True:
            line = readline(s)
            if line is None:
                if time.monotonic() - last > SILENCE_LIMIT_S:
                    if not started:
                        sys.exit("no answer from the instrument (wrong port, or firmware not running?)")
                    lines.append("# INCOMPLETE: the instrument stopped sending (recorded by the host)")
                    return lines, False
                continue
            last = time.monotonic()
            if not started:
                if line.startswith("# ERROR"):
                    sys.exit(line)
                if line == "# begin scan":
                    started = True
                    lines.append(line)
                continue
            lines.append(line)
            if line[:1] not in ("#", "E", ""):
                points += 1
                if points % 25 == 0:
                    print(f"\r  {points} points", end="", flush=True)
            if line == "# end scan":
                print(f"\r  {points} points")
                return lines, not any(l.startswith("# end:") and "complete" not in l for l in lines)
    except KeyboardInterrupt:
        print("\n  stopping the scan on the instrument ...")
        s.write(b"x\n")
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            line = readline(s)
            if line is None:
                continue
            if started:
                lines.append(line)
            if line == "# end scan":
                break
        else:
            lines.append("# INCOMPLETE: stopped from the host, no end line received")
        if not started:
            sys.exit("stopped before the scan began; nothing saved")
        return lines, False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("mode", choices=["run", "dummy", "info"])
    ap.add_argument("ohm", nargs="?", type=float, help="dummy-cell resistance, ohm")
    ap.add_argument("--name", help="file name YYYYMMDD_NNN (default: next free name of today)")
    ap.add_argument("--port", help="serial port, e.g. COM5")
    ap.add_argument("--out", type=Path, default=SCANS, help="output folder")
    a = ap.parse_args()
    if a.mode == "dummy" and not a.ohm:
        ap.error("dummy needs the resistance in ohm, e.g. dummy 10000")

    port = find_port(a.port)
    if a.mode == "info":
        with open_port(port) as s:
            show_info(s)
        return 0

    a.out.mkdir(parents=True, exist_ok=True)
    name = (a.name or next_free_name(a.out, dt.date.today().strftime("%Y%m%d"))).removesuffix(".csv")
    if not NAME.match(name):
        sys.exit(f"{name}: the name must be YYYYMMDD_NNN (PROTOCOL.md, Files)")
    path = a.out / f"{name}.csv"
    if path.exists():
        sys.exit(f"{path} exists; it is never overwritten")

    command = "run" if a.mode == "run" else f"dummy {a.ohm:.0f}"
    print(f"{command} on {port} -> {path}")
    with open_port(port) as s:
        lines, complete = record(s, command)

    stamp = f"# saved: {dt.datetime.now().isoformat(timespec='seconds')} by firmware/host/record.py from {port}"
    with open(path, "x", encoding="ascii", errors="replace", newline="\n") as f:
        f.write("\n".join([stamp] + lines) + "\n")

    try:
        shown = path.relative_to(ROOT)
    except ValueError:
        shown = path
    print(f"saved {shown}")
    for line in lines:
        if line.startswith(("# end:", "# dummy.fit", "# WARNING", "# INCOMPLETE")):
            print(" ", line[2:])
    print("  fill in status and notes for this file in data/potentiostat/manifest.csv")
    return 0 if complete else 2


if __name__ == "__main__":
    sys.exit(main())
