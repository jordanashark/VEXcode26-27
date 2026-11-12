# Robot Controls & Tuning Guide

Companion to `main.py` (four bar + claw, no pneumatics).

## Controller buttons

| Input | Action |
|---|---|
| Left stick Y (Axis 3) | Drive forward / back |
| Left stick X (Axis 4) | Turn (Single Arcade) |
| Right stick X (Axis 1) | Turn (Split Arcade) |
| **A** | Toggle Single / Split Arcade |
| **R1** (hold) | Four bar up |
| **R2** (hold) | Four bar down |
| **L1** (hold) | Claw close, then it keeps a light grip on release |
| **L2** (hold) | Claw open |

Release R1/R2 and the four bar holds its position. Stick inputs ramp up smoothly but stop instantly.

## Brain screen

- Line 3: `X, Y, H` (field position in inches, heading in degrees)
- Line 4: four bar and claw encoder positions

## Autonomous commands

| Command | What it does |
|---|---|
| `drive_distance(in)` | Profiled straight drive; negative = reverse |
| `turn_to(deg)` | Turn to an absolute heading |
| `turn_by(deg)` | Turn by a relative amount |
| `move_to_point(x, y)` | Face a point, then drive to it |
| `four_bar_goto(deg)` | Move four bar to a position, then hold |
| `claw_close()` / `claw_open()` | Run until the claw stalls |

Start pose is (0, 0) facing heading 0. Heading is clockwise-positive.

## Tuning order

Do these in order. Each step depends on the one before.

### 1. Check the setup
- **Directions:** If the claw closes on "open", flip the claw's reversed flag. If the four bar goes down on R1, flip its reversed flag.
- **IMU:** Keep the robot still during startup while it calibrates.

### 2. Measure the geometry
- `WHEEL_DIAMETER`, `DRIVE_GEAR_RATIO`: measure, don't guess.
- `TRACK_WIDTH`: run `turn_by(3600)` (10 spins). If the robot ends up facing a different way, adjust and repeat. This mainly affects the wheel-based heading estimate.
- **Distance check:** run `drive_distance(48)` against a tape measure. Error scales with wheel diameter.

### 3. Turn PID
Test with `turn_to(90)`, `turn_to(180)`, `turn_to(0)`.

| Symptom | Fix |
|---|---|
| Slow, stops short | Raise `TURN_KP` |
| Overshoots / oscillates | Lower `TURN_KP`, raise `TURN_KD` |
| Creeps the last degree | Raise `TURN_KI` slightly |
| Jerky start | Lower `TURN_SLEW_PER_S` |

### 4. Drive
Test with `drive_distance(24)` and `drive_distance(-24)`.

| Symptom | Fix |
|---|---|
| Lags behind the profile | Raise `DRIVE_KV` or `DRIVE_KP` |
| Wheels slip at start | Lower `DRIVE_ACCEL_IPS2` or `DRIVE_SLEW_PER_S` |
| Veers sideways | Raise `HEAD_KP` |
| Too slow overall | Raise `DRIVE_CRUISE_IPS` (about 0.65 of max by default) |

### 5. Four bar
- Read the bottom and top positions from the brain screen.
- Set `FOUR_BAR_MIN_DEG` and `FOUR_BAR_MAX_DEG`, then set `FOUR_BAR_LIMITS_ENABLED = True`.
- Tune with `four_bar_goto(250)` and back to `0`. Use the same table as turn PID with `FOUR_BAR_KP / KD`.
- **Sags under load:** add a little `FOUR_BAR_KI`.

### 6. Claw
- **Drops pieces:** raise `CLAW_HOLD_POWER`.
- **Motor gets hot:** lower `CLAW_HOLD_POWER`.
- **Stops too early or never stops:** adjust `CLAW_STALL_DPS` and `CLAW_STALL_MS`.

### 7. Settle logic
- **Moves end too slowly:** loosen `*_TOL` or shorten `*_SETTLE_MS`.
- **Moves end while still drifting:** tighten `*_SETTLE_VEL`.

## Using the log

After a run, pull `auton_log.csv` off the SD card (or read the terminal output) and graph **error vs. time** per move.

- **Error oscillates:** `kP` too high or `kD` too low.
- **Error plateaus above zero:** needs a bit of `kI`.
- **Error spikes early:** accel or slew too aggressive.
- **`*_timeout` rows:** the move never settled. Loosen tolerance or retune.
- **Pose jumps while the robot sits still:** wheel slip detected. Check the IMU and the `SLIP_THRESHOLD_DEG` setting.

## Before every match

- Charged battery.
- Robot placed in the same start position each time.
- Four bar fully down at power-on (position 0 = bottom).
- Wait for the IMU to finish calibrating before starting.