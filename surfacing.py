#!/usr/bin/env python3
"""XCarve Y-axis surfacing: jog to home, then run back-and-forth passes."""

import argparse
import os
import re
import select
import signal
import sys
import time
import threading
from math import ceil
from pathlib import Path
from typing import Optional

try:
    import tomllib
except ImportError:
    try:
        import tomli as tomllib  # type: ignore
    except ImportError:
        sys.exit("Python < 3.11 requires tomli: pip install tomli")

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    import tty
    import termios
    HAS_TTY = True
except ImportError:
    HAS_TTY = False


# ─── Config ───────────────────────────────────────────────────────────────────

_REQUIRED: dict[str, list[str]] = {
    "bit":       ["diameter", "ipm", "plunge_rate"],
    "workpiece": ["width", "length", "depth"],
    "surfacing": ["overlap_pct", "entry_offset"],
    "machine":   ["safe_z", "units"],
}


def load_config(path: str) -> dict:
    try:
        with open(path, "rb") as f:
            cfg = tomllib.load(f)
    except FileNotFoundError:
        sys.exit(f"Config file not found: {path}")
    for section, keys in _REQUIRED.items():
        if section not in cfg:
            sys.exit(f"config.toml: missing [{section}] section")
        for key in keys:
            if key not in cfg[section]:
                sys.exit(f"config.toml: missing {section}.{key}")
    if cfg["machine"]["units"] not in ("mm", "in"):
        sys.exit("config.toml: machine.units must be \"mm\" or \"in\"")
    return cfg


def derived(cfg: dict) -> dict:
    bit  = cfg["bit"]
    wp   = cfg["workpiece"]
    surf = cfg["surfacing"]
    step_over = bit["diameter"] * (1 - surf["overlap_pct"] / 100)
    if step_over <= 0:
        sys.exit("overlap_pct must be < 100")
    entry = surf["entry_offset"]
    return {
        "step_over":  step_over,
        "num_passes": ceil((wp["width"] + entry) / step_over),
        "x_start":    -entry,
        "y_start":    -entry,
        "y_end":      wp["length"] + 0.5 * bit["diameter"],
    }


# ─── G-code generation ────────────────────────────────────────────────────────

def generate_gcode(cfg: dict, d: dict) -> str:
    bit      = cfg["bit"]
    wp       = cfg["workpiece"]
    mach     = cfg["machine"]
    units    = mach["units"]
    strategy = cfg["surfacing"].get("strategy", "zigzag")
    if strategy not in ("zigzag", "conventional", "climb"):
        sys.exit("config.toml: surfacing.strategy must be 'zigzag', 'conventional', or 'climb'")

    lines = [
        f"; XCarve surfacing — {d['num_passes']} passes,"
        f" step-over {d['step_over']:.3f} {units},"
        f" depth -{wp['depth']} {units},"
        f" strategy={strategy}",
        "G21" if units == "mm" else "G20",
        "G90", "G94",
        f"G0 Z{mach['safe_z']:.3f}",
    ]

    if strategy in ("conventional", "climb"):
        going_pos = (strategy == "climb")
        y_from = d["y_start"] if going_pos else d["y_end"]
        y_to   = d["y_end"]   if going_pos else d["y_start"]
        lines += [
            f"G0 X{d['x_start']:.3f} Y{y_from:.3f}",
            f"G1 Z{-wp['depth']:.3f} F{bit['plunge_rate']:.1f}",
        ]
        for i in range(d["num_passes"]):
            x = d["x_start"] + i * d["step_over"]
            if i > 0:
                lines += [
                    f"G0 Z{mach['safe_z']:.3f}",
                    f"G0 X{x:.3f} Y{y_from:.3f}",
                    f"G1 Z{-wp['depth']:.3f} F{bit['plunge_rate']:.1f}",
                ]
            lines.append(f"G1 Y{y_to:.3f} F{bit['ipm']:.1f}")
    else:  # zigzag
        lines += [
            f"G0 X{d['x_start']:.3f} Y{d['y_start']:.3f}",
            f"G1 Z{-wp['depth']:.3f} F{bit['plunge_rate']:.1f}",
        ]
        for i in range(d["num_passes"]):
            x = d["x_start"] + i * d["step_over"]
            if i > 0:
                lines.append(f"G1 X{x:.3f} F{bit['ipm']:.1f}")
            y = d["y_end"] if i % 2 == 0 else d["y_start"]
            lines.append(f"G1 Y{y:.3f} F{bit['ipm']:.1f}")

    lines += [
        f"G0 Z{mach['safe_z']:.3f}",
        "G0 X0.000 Y0.000",
        "M2",
    ]
    return "\n".join(lines) + "\n"


# ─── GRBL helpers ─────────────────────────────────────────────────────────────

def _grbl_wake(ser) -> None:
    ser.write(b"\r\n\r\n")
    time.sleep(2)
    ser.flushInput()


def _grbl_send_wait(ser, line: str) -> str:
    ser.write((line.strip() + "\n").encode())
    buf = b""
    while True:
        buf += ser.readline()
        txt = buf.decode("ascii", errors="replace")
        if "ok" in txt or "error" in txt.lower():
            return txt.strip()


def _parse_wpos(text: str) -> Optional[tuple]:
    m = re.search(r"WPos:([-\d.]+),([-\d.]+),([-\d.]+)", text)
    if m:
        return float(m.group(1)), float(m.group(2)), float(m.group(3))
    mpos = re.search(r"MPos:([-\d.]+),([-\d.]+),([-\d.]+)", text)
    wco  = re.search(r"WCO:([-\d.]+),([-\d.]+),([-\d.]+)", text)
    if mpos and wco:
        return (
            float(mpos.group(1)) - float(wco.group(1)),
            float(mpos.group(2)) - float(wco.group(2)),
            float(mpos.group(3)) - float(wco.group(3)),
        )
    return None


# ─── Jog mode ─────────────────────────────────────────────────────────────────

_STEP_SIZES = {
    "mm": [0.1, 0.5, 1.0, 5.0, 10.0, 50.0],
    "in": [0.001, 0.005, 0.01, 0.05, 0.1, 0.5],
}
_JOG_FEED = {"mm": 500, "in": 20}


def _read_key_nb(fd: int) -> Optional[str]:
    r, _, _ = select.select([fd], [], [], 0)
    if not r:
        return None
    ch = os.read(fd, 1)
    if ch == b"\x1b":
        r2, _, _ = select.select([fd], [], [], 0.05)
        if r2:
            ch2 = os.read(fd, 1)
            if ch2 == b"[":
                r3, _, _ = select.select([fd], [], [], 0.05)
                if r3:
                    ch3 = os.read(fd, 1)
                    return "\x1b[" + ch3.decode("ascii", errors="replace")
        return "\x1b"
    return ch.decode("ascii", errors="replace")


def jog_to_home(ser, units: str, safe_z: float) -> None:
    if not HAS_TTY:
        print("Jog mode requires a Unix terminal — skipping.")
        return

    unit_cmd  = "G21" if units == "mm" else "G20"
    jog_feed  = _JOG_FEED[units]
    steps     = _STEP_SIZES[units]
    step_idx  = 2
    wpos      = (0.0, 0.0, 0.0)
    pos_lock  = threading.Lock()
    stop_evt  = threading.Event()

    _grbl_wake(ser)

    def _poll():
        nonlocal wpos
        while not stop_evt.is_set():
            try:
                ser.write(b"?")
                time.sleep(0.25)
                raw = b""
                while ser.in_waiting:
                    raw += ser.read(ser.in_waiting)
                    time.sleep(0.01)
                p = _parse_wpos(raw.decode("ascii", errors="replace"))
                if p:
                    with pos_lock:
                        wpos = p
            except Exception:
                pass

    threading.Thread(target=_poll, daemon=True).start()

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    print("\033[2J\033[H", end="", flush=True)
    print("=== JOG MODE ===")
    print("Arrow keys: X/Y jog   +/-: Z up/down   [/]: step size")
    print("H: set work-zero here   Q / Enter: done\n")

    try:
        tty.setraw(fd)
        while True:
            with pos_lock:
                x, y, z = wpos
            step = steps[step_idx]
            print(
                f"\r\033[K  WPos  X={x:9.3f}  Y={y:9.3f}  Z={z:9.3f}"
                f"   step={step} {units}  ",
                end="", flush=True,
            )
            time.sleep(0.05)
            key = _read_key_nb(fd)
            if key is None:
                continue
            if key in ("q", "Q", "\r", "\n"):
                break
            elif key in ("h", "H"):
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                _grbl_send_wait(ser, "G10 L20 P1 X0 Y0 Z0")
                print("\r\n  [Work zero set at current position]")
                tty.setraw(fd)
            elif key == "\x1b[A":  # Up arrow → +Y
                ser.write(f"$J={unit_cmd} G91 Y{step:.3f} F{jog_feed}\n".encode())
            elif key == "\x1b[B":  # Down arrow → -Y
                ser.write(f"$J={unit_cmd} G91 Y-{step:.3f} F{jog_feed}\n".encode())
            elif key == "\x1b[C":  # Right arrow → +X
                ser.write(f"$J={unit_cmd} G91 X{step:.3f} F{jog_feed}\n".encode())
            elif key == "\x1b[D":  # Left arrow → -X
                ser.write(f"$J={unit_cmd} G91 X-{step:.3f} F{jog_feed}\n".encode())
            elif key == "+":
                ser.write(f"$J={unit_cmd} G91 Z{step:.3f} F{jog_feed}\n".encode())
            elif key == "-":
                ser.write(f"$J={unit_cmd} G91 Z-{step:.3f} F{jog_feed}\n".encode())
            elif key == "]":
                step_idx = min(step_idx + 1, len(steps) - 1)
            elif key == "[":
                step_idx = max(step_idx - 1, 0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        stop_evt.set()
        try:
            ser.write(b"\x85")  # real-time jog cancel
        except Exception:
            pass

    unit_g = "G20" if units == "in" else "G21"
    _grbl_send_wait(ser, f"{unit_g} G90 G0 Z{safe_z:.3f}")
    print("\n  [Jog complete — at safe height]")


# ─── G-code streaming ─────────────────────────────────────────────────────────

def stream_gcode(ser, gcode: str, num_passes: int) -> None:
    sendable = [l for l in gcode.splitlines() if l.strip() and not l.strip().startswith(";")]
    pass_count = 0
    for lineno, line in enumerate(sendable, 1):
        resp = _grbl_send_wait(ser, line)
        if "error" in resp.lower() or "alarm" in resp.lower():
            print(f"\nGRBL error at line {lineno}: {resp}\n  Command: {line}")
            sys.exit(1)
        if line.startswith("G1 Y"):
            pass_count += 1
            print(f"\r  Pass {pass_count}/{num_passes}…", end="", flush=True)
    print("\nComplete.")


# ─── Port selection ───────────────────────────────────────────────────────────

def pick_port(cfg_port: Optional[str]) -> str:
    if cfg_port and cfg_port != "/dev/tty.usbserial-XXXX":
        return cfg_port
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        sys.exit("No serial ports found — check XController connection.")
    if len(ports) == 1:
        print(f"  Using port: {ports[0]}")
        return ports[0]
    for i, p in enumerate(ports):
        print(f"  [{i + 1}] {p}")
    try:
        return ports[int(input("Select port: ").strip()) - 1]
    except (ValueError, IndexError):
        sys.exit("Invalid selection.")


def _estimate_minutes(cfg: dict, d: dict) -> float:
    y_span = d["y_end"] - d["y_start"]
    dist   = d["num_passes"] * y_span + (d["num_passes"] - 1) * d["step_over"]
    return dist / cfg["bit"]["ipm"]


# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    parser = argparse.ArgumentParser(description="XCarve Y-axis surfacing tool")
    parser.add_argument("--config",   default="config.toml", help="Path to TOML config")
    parser.add_argument("--port",     help="Serial port (overrides config)")
    parser.add_argument("--dry-run",  action="store_true", help="Print G-code only")
    parser.add_argument("--jog",      action="store_true", help="Connect and enter jog mode only (no surfacing)")
    args = parser.parse_args()

    cfg   = load_config(args.config)
    units = cfg["machine"]["units"]

    if args.jog:
        if not HAS_SERIAL:
            sys.exit("Install pyserial first: pip install pyserial")
        port = args.port or pick_port(cfg.get("machine", {}).get("port"))
        baud = cfg["machine"].get("baud", 115200)
        try:
            with serial.Serial(port, baud, timeout=2) as ser:
                jog_to_home(ser, units, cfg["machine"]["safe_z"])
        except KeyboardInterrupt:
            print("\nAborted.")
        return

    d     = derived(cfg)

    print(f"\nSurfacing plan ({units}):")
    print(f"  Passes    : {d['num_passes']}")
    print(f"  Step-over : {d['step_over']:.3f}  "
          f"({cfg['surfacing']['overlap_pct']}% overlap on ⌀{cfg['bit']['diameter']} bit)")
    print(f"  Y travel  : {d['y_end'] - d['y_start']:.3f} per pass")
    print(f"  Plunge    : -{cfg['workpiece']['depth']} {units}")
    print(f"  Strategy  : {cfg['surfacing'].get('strategy', 'zigzag')}")
    print(f"  Est. time : ~{_estimate_minutes(cfg, d):.1f} min")

    gcode = generate_gcode(cfg, d)

    if args.dry_run:
        print("\n--- G-code ---\n")
        print(gcode)
        return

    print("\n  [1] Save G-code to file")
    print("  [2] Jog to home, then run on machine")
    print("  [3] Jog only")
    choice = input("Choice: ").strip()

    if choice == "3":
        if not HAS_SERIAL:
            sys.exit("Install pyserial first: pip install pyserial")
        port = args.port or pick_port(cfg.get("machine", {}).get("port"))
        baud = cfg["machine"].get("baud", 115200)
        try:
            with serial.Serial(port, baud, timeout=2) as ser:
                jog_to_home(ser, units, cfg["machine"]["safe_z"])
        except KeyboardInterrupt:
            print("\nAborted.")
        return

    if choice == "1":
        fname = input("Filename [surfacing.nc]: ").strip() or "surfacing.nc"
        Path(fname).write_text(gcode)
        print(f"Saved: {fname}")
        return

    if choice != "2":
        sys.exit("Cancelled.")

    if not HAS_SERIAL:
        sys.exit("Install pyserial first: pip install pyserial")

    port = args.port or pick_port(cfg.get("machine", {}).get("port"))
    baud = cfg["machine"].get("baud", 115200)

    try:
        with serial.Serial(port, baud, timeout=2) as ser:
            jog_to_home(ser, units, cfg["machine"]["safe_z"])
            print("\nReady to run. Press Enter to start, Ctrl-C to abort.")
            input()
            print("Running surfacing passes…")
            stream_gcode(ser, gcode, d["num_passes"])
    except KeyboardInterrupt:
        print("\nAborted.")


if __name__ == "__main__":
    main()
