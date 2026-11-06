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


_LOG = None  # optional file handle for a serial transcript


def _log(direction: str, text: str) -> None:
    if _LOG:
        _LOG.write(f"{time.strftime('%H:%M:%S')} {direction} {text}\n")
        _LOG.flush()


def _grbl_send_wait(ser, line: str) -> str:
    ser.write((line.strip() + "\n").encode())
    _log(">>", line.strip())
    buf = b""
    silent_since = time.time()
    while True:
        chunk = ser.readline()
        if chunk:
            text = chunk.decode("ascii", errors="replace").rstrip()
            _log("<<", text)
            silent_since = time.time()
            if text.startswith("<"):
                # Status report we asked for below — show it, don't treat as a reply.
                print(f"\n  [GRBL status: {text}]")
                continue
        elif time.time() - silent_since > 30:
            # No single move here takes >~15s, so 30s of silence means we're stuck.
            # Ask GRBL what state it's in (Idle / Run / Hold / Alarm / Door).
            print(f"\n  [no reply from GRBL for 30s while waiting on: {line.strip()}]")
            ser.write(b"?")
            _log(">>", "?")
            silent_since = time.time()
        buf += chunk
        txt = buf.decode("ascii", errors="replace")
        low = txt.lower()
        if "ok" in low or "error" in low or "alarm" in low:
            return txt.strip()


def _abort_machine(ser) -> None:
    """Stop motion now and discard GRBL's queued moves (they'd otherwise keep running)."""
    try:
        ser.write(b"!")      # feed hold: decelerate to a stop
        time.sleep(0.5)
        ser.write(b"\x18")   # soft reset: flush planner (keeps work zero)
        _log("!!", "aborted by user: feed hold + soft reset")
    except Exception:
        pass
    print("\nAborted — machine stopped (feed hold + reset). Spindle/vac are not controlled; turn them off.")


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
    state     = "?"          # GRBL state from last status report (Idle/Jog/Hold/Alarm…)
    last_seen = time.time()  # when we last got a status report
    poll_err  = ""
    pos_lock  = threading.Lock()
    ser_lock  = threading.Lock()  # one reader at a time: poll thread vs. send/wait
    stop_evt  = threading.Event()

    _grbl_wake(ser)

    def _poll():
        nonlocal wpos, state, last_seen, poll_err
        while not stop_evt.is_set():
            try:
                with ser_lock:
                    ser.write(b"?")
                    time.sleep(0.25)
                    raw = b""
                    while ser.in_waiting:
                        raw += ser.read(ser.in_waiting)
                        time.sleep(0.01)
                txt = raw.decode("ascii", errors="replace")
                for ln in txt.splitlines():
                    ln = ln.strip()
                    if ln and not ln.startswith("<") and ln != "ok":
                        _log("<<", ln)  # errors / alarms / messages from jog commands
                p = _parse_wpos(txt)
                m = re.search(r"<([A-Za-z]+(?::\d+)?)", txt)
                with pos_lock:
                    if p:
                        wpos = p
                    if m:
                        if m.group(1) != state:
                            _log("--", f"jog state {m.group(1)}")
                        state = m.group(1)
                        last_seen = time.time()
                    poll_err = ""
            except Exception as e:
                with pos_lock:
                    if str(e) != poll_err:
                        _log("!!", f"poll error: {e}")
                    poll_err = str(e)
                time.sleep(0.25)

    poll_thread = threading.Thread(target=_poll, daemon=True)
    poll_thread.start()

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    print("\033[2J\033[H", end="", flush=True)
    print("=== JOG MODE ===")
    print("Arrow keys: X/Y jog   +/-: Z up/down   [/]: step size")
    print("H: set work-zero here   Q / Enter: done   Ctrl-C: abort\n")

    def _jog(cmd: str) -> None:
        ser.write(cmd.encode())
        _log(">>", cmd.strip())

    was_unresponsive = False

    try:
        tty.setraw(fd)
        while True:
            with pos_lock:
                x, y, z = wpos
                st, age, err = state, time.time() - last_seen, poll_err
            step = steps[step_idx]
            if err:
                status = f"SERIAL ERROR: {err}"
            elif age > 2:
                status = f"NO RESPONSE {age:.0f}s (last: {st})"
            else:
                status = st
            if (age > 2) != was_unresponsive:
                was_unresponsive = age > 2
                _log("--", f"jog: {'NO RESPONSE from GRBL' if was_unresponsive else 'GRBL responding again'}")
            print(
                f"\r\033[K  WPos  X={x:9.3f}  Y={y:9.3f}  Z={z:9.3f}"
                f"   step={step} {units}   [{status}]",
                end="", flush=True,
            )
            time.sleep(0.05)
            key = _read_key_nb(fd)
            if key is None:
                continue
            _log("--", f"key {key!r}")
            if key == "\x03":  # Ctrl-C arrives as a byte in raw mode, not SIGINT
                raise KeyboardInterrupt
            if key in ("q", "Q", "\r", "\n"):
                break
            elif key in ("h", "H"):
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                with ser_lock:  # keep the poll thread from eating our "ok"
                    ser.reset_input_buffer()
                    _grbl_send_wait(ser, "G10 L20 P1 X0 Y0 Z0")
                print("\r\n  [Work zero set at current position]")
                tty.setraw(fd)
            elif key == "\x1b[A":  # Up arrow → +Y
                _jog(f"$J={unit_cmd} G91 Y{step:.3f} F{jog_feed}\n")
            elif key == "\x1b[B":  # Down arrow → -Y
                _jog(f"$J={unit_cmd} G91 Y-{step:.3f} F{jog_feed}\n")
            elif key == "\x1b[C":  # Right arrow → +X
                _jog(f"$J={unit_cmd} G91 X{step:.3f} F{jog_feed}\n")
            elif key == "\x1b[D":  # Left arrow → -X
                _jog(f"$J={unit_cmd} G91 X-{step:.3f} F{jog_feed}\n")
            elif key == "+":
                _jog(f"$J={unit_cmd} G91 Z{step:.3f} F{jog_feed}\n")
            elif key == "-":
                _jog(f"$J={unit_cmd} G91 Z-{step:.3f} F{jog_feed}\n")
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

    # Make sure the poll thread is gone and stale jog replies are cleared before
    # we start waiting on our own "ok".
    poll_thread.join(timeout=2)
    time.sleep(0.3)
    ser.reset_input_buffer()
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
            # GRBL acks a line when it is queued (~15-move planner), not when it runs,
            # so this counter is a few passes ahead of the actual cutter.
            print(f"\r  Queued pass {pass_count}/{num_passes}…", end="", flush=True)
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


# GRBL motion limits in GRBL's own units (mm/min and mm/s²), i.e. the $110-$112 and
# $120-$122 settings. Defaults are the X-Carve / X-Controller stock values; override
# under [machine] in config.toml if yours differ. Checked against a real 71-pass job
# (2026-11-05): predicted 18.45 s/pass vs. 18.38 s measured.
_MOTION_DEFAULTS = {
    "max_rate_xy": 8000.0,  # $110/$111 mm/min (G0 rapid speed in X/Y)
    "max_rate_z":  500.0,   # $112      mm/min (G0 rapid speed in Z — slow!)
    "accel_xy":    500.0,   # $120/$121 mm/s²
    "accel_z":     50.0,    # $122      mm/s²
}


def _move_seconds(dist: float, v: float, a: float) -> float:
    """Trapezoidal (or triangular) move from stop to stop. dist mm, v mm/s, a mm/s²."""
    if dist <= 0:
        return 0.0
    d_ramp = v * v / a  # distance spent accelerating + decelerating
    if dist >= d_ramp:
        return dist / v + v / a
    return 2 * (dist / a) ** 0.5  # never reaches full speed


def estimate_seconds(gcode: str, mach: dict) -> float:
    """Simulate the G-code the way GRBL executes it.

    Assumes the machine comes to a stop between moves. That holds for these toolpaths:
    every junction is a 90° or sharper corner.
    """
    lim = {k: float(mach.get(k, v)) for k, v in _MOTION_DEFAULTS.items()}
    max_v = (lim["max_rate_xy"] / 60, lim["max_rate_xy"] / 60, lim["max_rate_z"] / 60)
    max_a = (lim["accel_xy"], lim["accel_xy"], lim["accel_z"])

    scale = 25.4  # G20 default for safety; G21 switches to 1
    pos   = [0.0, 0.0, 0.0]
    feed  = 0.0   # mm/min
    mode  = "G0"
    total = 0.0
    for raw in gcode.splitlines():
        line = raw.split(";")[0].strip().upper()
        if not line:
            continue
        words = dict((w[0], w[1:]) for w in line.split())
        for g in re.findall(r"G(\d+)", line):
            if g == "20": scale = 25.4
            elif g == "21": scale = 1.0
            elif g in ("0", "1"): mode = "G" + g
        if "F" in words:
            feed = float(words["F"]) * scale
        target = list(pos)
        for i, ax in enumerate("XYZ"):
            if ax in words:
                target[i] = float(words[ax]) * scale
        delta = [t - p for t, p in zip(target, pos)]
        dist  = sum(c * c for c in delta) ** 0.5
        if dist == 0:
            continue
        unit = [abs(c) / dist for c in delta]
        # GRBL scales the move so no single axis exceeds its own limit.
        v_lim = min(max_v[i] / unit[i] for i in range(3) if unit[i] > 0)
        a_lim = min(max_a[i] / unit[i] for i in range(3) if unit[i] > 0)
        v = v_lim if mode == "G0" else min(feed / 60, v_lim)
        total += _move_seconds(dist, v, a_lim)
        pos = target
    return total


# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    parser = argparse.ArgumentParser(description="XCarve Y-axis surfacing tool")
    parser.add_argument("--config",   default="config.toml", help="Path to TOML config")
    parser.add_argument("--port",     help="Serial port (overrides config)")
    parser.add_argument("--dry-run",  action="store_true", help="Print G-code only")
    parser.add_argument("--jog",      action="store_true", help="Connect and enter jog mode only (no surfacing)")
    args = parser.parse_args()

    if not args.dry_run:
        print("\n  ⚠  Make sure the laptop is plugged into its charger (shared ground with the")
        print("     XController). On battery the controller's USB link locks up mid-job.")

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
    gcode = generate_gcode(cfg, d)
    est = estimate_seconds(gcode, cfg["machine"])
    print(f"  Est. time : ~{int(est // 60)} min {int(est % 60):02d} s"
          f"  (~{est / d['num_passes']:.1f} s/pass)")

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

    global _LOG
    log_path = Path(time.strftime("surfacing-%Y%m%d-%H%M%S.log"))
    _LOG = open(log_path, "w")
    print(f"  Logging serial traffic to {log_path}")

    with serial.Serial(port, baud, timeout=2) as ser:
        try:
            jog_to_home(ser, units, cfg["machine"]["safe_z"])
            print("\n  ▶ Turn ON the router (DWP611) and the shop vac now.")
            print("\nReady to run. Press Enter to start, Ctrl-C to abort.")
            input()
            print(f"Running surfacing passes… (est. finish {time.strftime('%H:%M', time.localtime(time.time() + est))})")
            stream_gcode(ser, gcode, d["num_passes"])
        except KeyboardInterrupt:
            _abort_machine(ser)


if __name__ == "__main__":
    main()
