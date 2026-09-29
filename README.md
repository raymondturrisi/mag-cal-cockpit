# magcc — Mag Cal Cockpit

Experimental and in active development. It works and is in use, but expect quirks and breaking changes between 0.x releases.

Live magnetometer hard/soft-iron calibration. A sensor streams IMU packets over UDP; magcc logs them while the sensor is in motion, fits a calibration (LM or LSE ellipsoid fit to the local WMMHR field strength), and shows the result in a browser UI.

## Install

```
pip install magcc          # or: pipx install magcc
```

Requires Python ≥ 3.11. This installs two commands, `magcc` and `magcc-replay`. If they aren't found on your `PATH`, use `python3 -m magcc` instead.

## Run

```
magcc                                  # listen on UDP 0.0.0.0:50100, UI on http://localhost:8080
magcc --lat 42.357 --lon -71.087       # fix the WMMHR location instead of using the browser's
magcc --out-dir ~/cal_runs             # where magcc_<timestamp>/ exports go (default: cwd)
magcc --inspect magcc_20260924_204908/magcc_session.json   # reload an exported session offline
magcc-replay magcc_20260924_204908/magcc_raw.csv --speed 3 # replay a CSV as magcc packets
```

| Option | Default | |
|---|---|---|
| `--udp-addr`, `--udp-port` | `0.0.0.0`, `50100` | UDP listen address |
| `--web-port` | `8080` | UI port |
| `--magcc` | `auto` | Packet schema: `auto` or `v0` |
| `--lat`, `--lon` | — | WMMHR location; overrides the browser |
| `--mount-roll/-pitch/-yaw` | `0` | Sensor mount angles (deg), display only |
| `--out-dir` | `.` | Export root |
| `--inspect` | — | Load a session JSON, no UDP |
| `--no-browser` | off | Don't open a browser |

### Field reference

The expected field strength, inclination and declination come from NOAA's WMMHR (`wmmhr` package) evaluated at today's date. Location is taken from, in order of precedence: `--lat/--lon` or the lat/lon typed into the UI, then the values saved in a session loaded with `--inspect`, then the browser's geolocation, then a Boston default. The UI shows which source is in use. Browsers only allow geolocation on `localhost` or HTTPS, so opening the UI from another machine by IP falls back to the default unless you set the location.

## Protocol (magcc v0)

One ASCII sentence per UDP datagram:

```
|ts,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw*
```

| Field | Units |
|---|---|
| `ts` | seconds (epoch or monotonic) |
| `ax..az` | any (not used by the fit) |
| `gx..gz` | rad/s (motion gating threshold) |
| `mx..mz` | Gauss, µT or nT (select the matching unit in the UI) |
| `roll, pitch, yaw` | rad (roll/pitch used for level-frame diagnostics) |

Exactly 13 fields; anything else is ignored. `hbk_cv7_cli --mcc` sends this format. All vectors are in the sensor frame. With `--magcc auto` (default) magcc locks onto the first schema that parses. Later schemas will start with a tag field (e.g. `|MAGCC1,...*`) so they remain distinguishable from v0.

## Sensor requirements

The AHRS/IMU application feeding magcc must:

- send magcc v0 packets (above) over UDP to the magcc host and port;
- send **raw, uncalibrated** magnetometer data. Disable or clear any onboard hard/soft-iron calibration first, otherwise magcc fits on top of it;
- send roll/pitch from its own filter (used only for the level-frame diagnostics, not the fit).

## Workflow

1. Start the sensor stream and run `magcc`. In the dashboard, check the location source and set **Mag Units** to match the sensor.
2. **START**, then rotate the sensor slowly through every orientation until the point cloud covers the whole sphere. Only samples with gyro rate above the **Gyro Motion Gating** threshold are logged. **PAUSE** when done.
3. **CALIBRATE** fits hard and soft iron (LM or LSE, set by **Method**) to the local field strength. The RLS view updates live while logging; the LM/LSE result is the final one.
4. Optional cleanup, then **CALIBRATE** again:
   - The **Field Magnitude & Region Editor** plot shows field magnitude over time. Right-click the bar under it to split the timeline at that point; drag split markers to adjust; **+ Split / − Split** add or remove splits.
   - Left-click a block to toggle it **ON/OFF**. Only ON blocks are used in the fit.
   - **Roll ≤ / Pitch ≤** sliders exclude samples beyond those angles.
   - The **Inspect** tab shows per-point time, attitude and error to find the samples worth cutting.
5. **EXPORT** writes `magcc_<timestamp>/` with `mag_cal.dat`, `magcc_raw.csv` (with a `used` column for the fit mask) and `magcc_session.json` (reload with `--inspect`).

## Output: `mag_cal.dat`

```
b = bx,by,bz            # hard iron, in the selected unit
A = a11,a12,...,a33     # soft iron, 3x3 symmetric, row-major
```

Apply as `m_cal = A @ (m_raw - b)`. The `#` header records the method, unit, sample count, fit residuals and expected field strength.

## Known quirks

- Reloading the page re-runs calibration on all samples and drops block/roll/pitch filters. Re-apply them before exporting.
- Calibrating with too little orientation coverage fails with a numpy complex-cast error instead of a clear message.
- The UI loads plotly and three.js from CDNs, so it needs internet access.
- The UI and its controls are open to anyone on the network, with no authentication.
- Without a location (no `--lat/--lon`, browser location denied), the field reference defaults to Boston.
- Only tested on macOS.
- Double-clicking a timeline split marker removes it.

## License

Copyright (C) 2026 Raymond Turrisi, Massachusetts Institute of Technology

Licensed under GPL-3.0-only. See the LICENSE file.

If you have been granted a license under the source-available "MIT Spurdog AUV" license, that license supersedes the GPL-3.0 license for your use of this software.
