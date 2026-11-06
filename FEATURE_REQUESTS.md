# Feature requests

## 1. Separate X/Y zero and Z zero

**Today:** in jog mode, `H` runs `G10 L20 P1 X0 Y0 Z0`, which sets X, Y and Z zero all at once at the current position. To get the XY corner right, the bit has to be at the workpiece corner. To get Z right, the bit has to be touching the surface. Doing both at one spot is awkward. Often the bit needs to touch off on the surface somewhere else, such as the highest point of uneven stock, or the corner sits off the edge where there is no surface to touch.

**Idea:** separate keys in jog mode, for example:

| Key | Action | G-code |
|---|---|---|
| `H` | Zero all (current behaviour) | `G10 L20 P1 X0 Y0 Z0` |
| `X` / `Y` | Zero just X, or just Y | `G10 L20 P1 X0` / `G10 L20 P1 Y0` |
| `C` (corner) | Zero X and Y only | `G10 L20 P1 X0 Y0` |
| `Z` | Zero Z only | `G10 L20 P1 Z0` |

**Things to explore:**
- **Display:** show in the jog display which axes have been zeroed this session (e.g. `X✓ Y✓ Z–`), and warn on exit if one hasn't been.
- **Touch-off:** the `Z` key could offer a paper-thickness offset (e.g. zero at +0.004").
- **Probe:** if a touch plate is added later, `G38.2` probing could replace manual Z zeroing.
- **Wording:** keep the confirmation messages clear about which axes were set, so a mistake like zeroing Z in the air is obvious.

## 2. Raise/lower Z after jogging to fit the vacuum boot

**Today:** after jog mode exits, the script moves to `safe_z` and waits for Enter. With the bit that close to the stock, there isn't room to attach the X-Carve dust boot.

**Idea:** after zeroing and before the "turn on router + vac / press Enter" prompt, offer a step to move Z up so the boot can be attached, then come back down:

1. Prompt: `Raise Z to attach dust boot? [Y/n]`
2. Move to a configurable clearance height, e.g. `machine.boot_z = 1.5` in config.toml, using machine coordinates (`G53 G0 Z…`) or work coordinates.
3. Wait: `Attach the dust boot, then press Enter`.
4. Return to `safe_z` and continue to the start prompt.

**Things to explore:**
- **Work zero:** it must survive this step. Use only absolute moves in work coordinates (G90) and never re-zero here.
- **Clearance limit:** check that `boot_z` is within Z travel. There are no limit switches, so GRBL won't stop an over-travel; it would stall or skip steps. Consider capping it at a configured `z_travel_max`.
- **Jog fallback:** let the user jog Z with `+`/`-` in this step too, instead of a fixed height, for stock of varying thickness.
- **Combine with #1?** A sequence of zero XY at the corner, raise and attach the boot, then zero Z elsewhere might want both features working together.
