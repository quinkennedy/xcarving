# XCarve Surfacing Tool

Interactive CLI for surfacing/flattening stock on an XCarve with XController (GRBL).
Runs back-and-forth passes along the Y axis, stepping over in X between passes.
Includes a live jog mode to position the bit before cutting.

## Requirements

- Python 3.9+
- **pyserial** — serial communication with the XController
- **tomli** — TOML config parsing (only needed on Python < 3.11; built-in on 3.11+)

```
pip install pyserial tomli
```

## Before you connect: plug the laptop into power

**Keep the laptop on its charger whenever it's connected to the XController.** The charger gives the laptop and the controller a shared ground. On battery, the laptop's USB ground floats relative to the controller, and the XController's USB-serial chip locks up, even with no motion. It typically locks up within seconds to a minute. When that happens:

- the machine stops mid-job or stops responding to jogs (green light still on, motors still energized)
- the script shows `[NO RESPONSE …]` in jog mode, or waits forever while streaming
- reopening the port fails with `termios.error: (22, 'Invalid argument')`

Unplugging the USB cable doesn't clear the lockup. Power-cycle the controller.

## Configuration

Edit `config.toml` before each job. All dimensions use the unit set in `machine.units`.

```toml
[bit]
diameter    = 6.35   # bit diameter
ipm         = 1000   # cutting feed rate (mm/min or in/min)
plunge_rate = 100    # plunge feed rate

[workpiece]
width  = 200   # X dimension (how far to step across)
length = 300   # Y dimension (length of each pass)
depth  = 1.5   # how deep to cut (positive number)

[surfacing]
overlap_pct  = 40   # % overlap between passes
                    # 40% overlap → step-over = 60% of bit diameter
entry_offset =  5   # distance off the workpiece corner (-X and -Y)
                    # to position before plunging
strategy     = "zigzag"  # "zigzag" (bidirectional), "conventional" (+Y only), "climb" (-Y only)

[machine]
safe_z = 5                          # clearance height for rapid moves
units  = "mm"                       # "mm" or "in"
port   = "/dev/tty.usbserial-XXXX"  # serial port; leave as placeholder to be prompted
baud   = 115200
```

### Time estimate

The estimate simulates every move the way GRBL runs it: cuts at the feed rate, plunges at the plunge rate, rapids at the machine's maximum speeds, with acceleration and deceleration on every move. It uses the stock X-Carve / X-Controller motion limits. If you've changed your GRBL settings, add any of these optional keys under `[machine]`:

```toml
max_rate_xy = 8000   # $110/$111, mm/min
max_rate_z  = 500    # $112, mm/min (Z rapids are slow: ~20 in/min)
accel_xy    = 500    # $120/$121, mm/s²
accel_z     = 50     # $122, mm/s²
```

These are always in GRBL's units (mm/min, mm/s²), whatever `units` is set to. On a 71-pass job the estimate came out 22:01 against 21:49 measured.

### How step-over is calculated

```
step_over = diameter × (1 − overlap_pct / 100)
num_passes = ceil((width + entry_offset) / step_over)
```

A 40% overlap on a 6.35 mm bit → 3.81 mm step-over → 54 passes for a 200 mm wide workpiece.

### Cutting strategies

| Value | Behaviour |
|---|---|
| `zigzag` | Bidirectional — alternates +Y / -Y each pass, stepping over at cutting depth (default) |
| `conventional` | Unidirectional +Y — retracts to safe Z and rapids back to Y start between each pass |
| `climb` | Unidirectional -Y — retracts to safe Z and rapids back to Y end between each pass |

For `conventional` and `climb` each pass gets its own plunge; for `zigzag` there is a single plunge at the start.

### Y-axis travel

The tool center starts at `−entry_offset` on the near side and travels to `workpiece.length + 0.5 × bit.diameter` on the far side, so the cutting edge reaches both edges of the stock.

## Usage

```
python3 surfacing.py [--config config.toml] [--port /dev/tty.xxx] [--dry-run]
```

| Flag | Description |
|---|---|
| `--config FILE` | Path to TOML config (default: `config.toml`) |
| `--port PORT` | Serial port, overrides value in config |
| `--dry-run` | Print generated G-code to stdout, no serial connection |
| `--jog` | Connect and enter jog mode only — no surfacing run |

### Option 1 — Save G-code to file

Choose **[1]** at the prompt to write a `.nc` file you can load in UGS, CNCjs, or any GRBL sender.

### Option 2 — Jog to home, then run

Choose **[2]** to connect to the machine and use the interactive jog mode.

### Option 3 — Jog only

Choose **[3]** (or pass `--jog` on the command line) to connect and enter jog mode without running any surfacing. Useful for positioning the bit, setting work zero, or manual moves.

#### Coordinate convention

The script treats the **bottom-left corner of the workpiece at the surface** as the work origin (X=0, Y=0, Z=0). Home the machine to this point in jog mode before cutting.

#### Jog mode controls

| Key | Action |
|---|---|
| Arrow keys | Jog X / Y |
| `+` | Jog Z up |
| `-` | Jog Z down |
| `]` | Increase step size |
| `[` | Decrease step size |
| `H` | Set work zero at current position |
| `Q` or Enter | Exit jog mode and proceed |

Step sizes cycle through: `0.1 → 0.5 → 1.0 → 5.0 → 10.0 → 50.0` (mm) or `0.001 → 0.005 → 0.01 → 0.05 → 0.1 → 0.5` (in).

#### Cutting sequence

Once you confirm the home position and press Enter to start:

1. Rapid to safe Z
2. Rapid to X=-`entry_offset`, Y=-`entry_offset` (off the workpiece corner)
3. Plunge to cutting depth
4. Run back-and-forth Y passes, stepping over in X at cutting depth between passes
5. Retract to safe Z and return to X=0, Y=0

The single plunge happens off the workpiece so the bit enters the material from the side rather than plunging straight in.
