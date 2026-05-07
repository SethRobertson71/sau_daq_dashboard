---
name: svss-python-stack
description: >
  Engineering skill for developing, debugging, and maintaining the Stationary
  Vehicle Sensor Suite (SVSS) Python software stack. Use this skill whenever
  working on config.py, acquisition.py, dashboard.py, or t7_drivetrain_control.lua
  for this project. Also triggers for: LJM connectivity issues (Tailscale/IP),
  I2C bus errors (LJMi2c/NAU7802/ICM20948), PyQt6/pyqtgraph dashboard bugs,
  SensorTag/ChannelConfig changes, stepper motor Lua control loop issues,
  pin mapping changes, new sensor integration, calibration, filtering, or any
  debugging session involving T7 hardware over Ethernet. If the user is working
  on anything touching the LabJack T7, the SVSS dashboard, or the drivetrain
  Lua script, use this skill immediately.
---

# SVSS Python Stack — Engineering Skill
## Codebase Version: v2.6.2
## Last updated: patch_config_channel bugfix session (Engineer panel calibration persistence)

---

## ⚠️ SELF-UPDATE PROTOCOL — READ THIS FIRST

**Every time code changes are made to any SVSS file, this skill file must be
updated before the session ends.**

### Trigger conditions
Any of the following require a skill update:
- New class, method, or function added to any of the four source files
- Constructor signature changed (new parameters, removed parameters)
- Import contract changed (new names exported, old names removed)
- Critical bug fixed that was previously documented as correct behaviour
- New DIO-EF, I2C, or LJM register usage pattern established
- New UI widget or control added to dashboard.py
- Filter system extended (new FilterProfile value, new algorithm)
- Any section of this file found to be incorrect during a session

### How to update
At the end of any session where code changed:
1. Read the modified source file(s) from `/mnt/user-data/outputs/`
2. Extract updated signatures, imports, and patterns
3. Update the relevant sections of this file — do **not** append; replace stale
   content in-place so the file stays concise
4. Write the updated file to `/mnt/skills/user/svss-python-stack/SKILL.md`
5. Confirm the write succeeded before closing the session

If the skill file path is read-only in the current environment, write the
updated file to `/mnt/user-data/outputs/SKILL.md` and tell the user to copy
it to `/mnt/skills/user/svss-python-stack/SKILL.md` manually.

---

## Quick Reference

| File | Role | Change Frequency |
|---|---|---|
| `config.py` | Hardware pin map, SensorTag enum, ChannelConfig lists | Each new sensor or pin change |
| `acquisition.py` | FilterProfile/ChannelFilter, LJM I2C drivers, SensorReader subclasses | Bug fixes, new sensors, filter changes |
| `dashboard.py` | PyQt6 UI, pyqtgraph plots, CSV logger, per-card filter/zero controls | UI changes, filter UI |
| `t7_drivetrain_control.lua` | T7 stepper P-controller (Lua 5.1) | Control tuning only |

**T7 IP:** `192.168.1.10` | **LJM open call:** `ljm.openS("T7", "ETHERNET", "192.168.1.10")`
> ⚠️ Never use `"ANY"` as the identifier — Tailscale intercepts UDP broadcasts.

---

## Architecture Overview

```
Host PC (Python)
├── config.py          ← SensorTag enum, ChannelConfig lists, all hardware constants
├── acquisition.py     ← FilterProfile, ChannelFilter, LJMi2c, NAU7802, ICM20948,
│   │                     SensorReader subclasses, AcquisitionThread
│   └── AcquisitionThread → ChannelFilter (per channel) → DataFrame → RingBuffer
└── dashboard.py       ← PyQt6 + pyqtgraph, reads RingBuffer, logs CSV
                          ChannelValueCard: display-layer ChannelFilter + Zero button

Data flow:
  NAU7802/AIN read_frame()
    → raw × scale + offset          (calibration, SensorReader)
    → ChannelFilter.push()          (acquisition layer — affects CSV)
    → DataFrame → RingBuffer → CSV  ← logged here (acq-filtered only)
    → ChannelValueCard.update_value()
        → display ChannelFilter     (display layer — card + plot only)
        → − display_offset          (Zero button)
        → returns display value     → plot_frame → plots

T7 Hardware (Ethernet)
├── AIN0–3            ← Linear potentiometers (suspension mode)
├── EIO0/1,2/3,4/5,6/7 ← NAU7802 I2C buses (SDA/SCL pin pairs)
├── CIO0/1            ← ICM-20948 I2C (IMU)
├── CIO2/3            ← Rotary encoder I2C (steering mode)
├── FIO0              ← Hall effect RPM (DIO-EF Index 3, active-low sensor)
├── FIO2/3            ← Quadrature encoder (DIO-EF Index 10, steering mode)
├── FIO4              ← Stepper STEP pulse (DIO-EF PWM output)
├── FIO5              ← Stepper DIR digital output
└── USER_RAM0_F32     ← Python writes measured RPM → Lua reads for control

T7 Lua (t7_drivetrain_control.lua)
└── Reads USER_RAM0_F32 → P-controller → drives FIO4/FIO5 stepper
```

---

## Critical Rules — Read First

### 1. LJM Connection (Always Direct IP)
```python
from labjack import ljm
handle = ljm.openS("T7", "ETHERNET", "192.168.1.10")
```
Never use `"ANY"` — Tailscale blocks UDP discovery.

### 2. LJM I2C API (Correct vs. Wrong)
```python
# ✅ CORRECT — array register names
ljm.eWriteNameByteArray(handle, "I2C_DATA_TX", n, tx_bytes)
ljm.eReadNameByteArray(handle, "I2C_DATA_RX", n)

# ❌ WRONG — individually indexed names cause LJME_INVALID_NAME (error 1294)
ljm.eWriteName(handle, "I2C_DATA_TX0", value)
```

### 3. DIO-EF Configuration — CRITICAL, PREVIOUSLY BUGGY

**`DIO#_EF_ENABLE` is NOT the mode/index register. It is boolean: 0 or 1 only.**
Writing the index number (e.g. `3`) to ENABLE silently enables the EF with
the default/prior index (usually 0 = PWM output), producing no measurement.

**Correct register is `DIO#_EF_INDEX`.** Mandatory sequence:

```python
# Frequency In (Hall RPM) — Index 3, rising edges
ljm.eWriteName(h, f"DIO{pin}_EF_ENABLE",   0)   # 1: disable first
ljm.eWriteName(h, f"DIO{pin}_EF_INDEX",    3)   # 2: set index while disabled
ljm.eWriteName(h, f"DIO{pin}_EF_CONFIG_A", 2)   # 3: continuous mode (bit1=1)
ljm.eWriteName(h, f"DIO{pin}_EF_ENABLE",   1)   # 4: enable (1, not 3!)

# Quadrature Encoder — Index 10
ljm.eWriteName(h, f"DIO{a}_EF_ENABLE", 0)
ljm.eWriteName(h, f"DIO{b}_EF_ENABLE", 0)
ljm.eWriteName(h, f"DIO{a}_EF_INDEX",    10)
ljm.eWriteName(h, f"DIO{a}_EF_CONFIG_A",  0)    # count offset = 0
ljm.eWriteName(h, f"DIO{a}_EF_ENABLE",    1)    # B pin auto-paired
```

**`DIO#_EF_READ_A` returns period in 80 MHz clock TICKS, not Hz, not µs.**
```python
ticks   = ljm.eReadName(handle, f"DIO{RPM_PIN}_EF_READ_A")
freq_hz = 80_000_000.0 / ticks     # ticks=0 → no edges → return 0.0
rpm     = (freq_hz / RPM_PULSES_PER_REV) * 60.0
```

**Active-low Hall sensor note:** Sensor output is normally HIGH (~3.2V),
pulses LOW briefly per revolution. Index 3 triggers on rising edges — the
rising edge at the trailing edge of each low pulse is still one event per
revolution. No polarity inversion needed. No external pull-up resistor is
used (3.3V supply, sensor has internal pull).

### 4. Lua 5.1 Constraints on T7
| Limit | Value |
|---|---|
| Local variables per function | 50 |
| Upvalues per inner function | 10 |
| Script size (code-only bytes) | 4,096 |
| Bitwise operators | None — use `bit.band()`, `bit.bor()`, `bit.lshift()` |

Current Lua script: **~4,078 bytes**. Check before any addition:
`awk 'NF && !/^--/' t7_drivetrain_control.lua | wc -c`

### 5. SensorTag ↔ ChannelConfig Invariant
`ChannelConfig.register` holds a `SensorTag` integer value — not a Modbus
address. Validate with:
```python
valid = {t.value for t in SensorTag}
bad = [ch for ch in channels if ch.register not in valid]
assert not bad, f"Invalid SensorTag values: {[c.key for c in bad]}"
```

### 6. Dashboard Import Contract — Current (do not change these names)
```python
from config import (
    TestMode, ChannelConfig, MODE_CHANNELS, COLOR_CYCLE,
    T7_IP_ADDRESS, T7_CONNECTION,
    SAMPLE_RATE_HZ_DEFAULT, SAMPLE_RATE_HZ_MIN, SAMPLE_RATE_HZ_MAX,
    PLOT_HISTORY_SECONDS, BUFFER_MAXLEN, PLOT_REFRESH_MS,
    CSV_DEFAULT_DIR, APP_TITLE, APP_VERSION,
)
from acquisition import (
    AcquisitionThread, RingBuffer, CSVLogger, AcqStatus, DataFrame,
    FilterProfile, ChannelFilter,
)
```

---

## Filter System (acquisition.py)

### FilterProfile Enum
```python
class FilterProfile(Enum):
    NONE       = "None"       # passthrough
    SMA        = "SMA"        # Simple Moving Average — active
    MEDIAN_EMA = "Median+EMA" # stub — falls back to SMA until implemented
```

### ChannelFilter Class
```python
class ChannelFilter:
    def __init__(
        self,
        profile:   FilterProfile = FilterProfile.NONE,
        sma_n:     int   = 5,      # window length, min 2
        ema_alpha: float = 0.25,   # for future EMA stage
    ): ...

    def push(self, raw: float) -> float: ...  # feed sample, get filtered output
    def reset(self): ...                       # clear state on stream start/stop
```

**SMA implementation note:** Returns the window mean from sample 1 (no
zero-padding). Window fills progressively — no startup step artifact.

### Implementing MEDIAN_EMA (when ready)
Replace the stub in `ChannelFilter.push()` for `MEDIAN_EMA`:
```python
if self.profile == FilterProfile.MEDIAN_EMA:
    median_val = sorted(self._window)[(len(self._window) - 1) // 2]
    return self._ema(median_val)
```
Then add `MEDIAN_EMA` to `_FILTER_OPTIONS` in `ChannelValueCard` and
`_filt_options` in `MainWindow._build_ui()`. No other changes needed.

### Filter Layering
Two independent filter layers. Both use `ChannelFilter`:

**Acquisition layer** (in `_acquire_loop`, affects CSV):
- One `ChannelFilter` per channel, instantiated at stream start
- Profile set by sidebar "Acq Filter (CSV)" combo + N spinbox
- Passed to `AcquisitionThread` as `filter_profile` and `filter_n`

**Display layer** (in `ChannelValueCard`, card + plot only):
- One `ChannelFilter` per card, independent from acquisition filter
- Profile set by per-card dropdown (`None` / `SMA 5` / `SMA 10`)
- `reset()` called on stream stop via `card.reset_zero()`

---

## acquisition.py — Current Class Hierarchy

```
FilterProfile (Enum)        — NONE / SMA / MEDIAN_EMA(stub)
ChannelFilter               — stateful filter, push(raw)→float
AcqStatus (Enum)            — DISCONNECTED/CONNECTING/CONNECTED/STREAMING/ERROR/STOPPED
DataFrame (dataclass)       — wall_time, t7_timestamp_ms, values: Dict[str,float]
RingBuffer                  — thread-safe deque, push()/drain()
LJMi2c                      — soft I2C bus driver, bound to one (SDA,SCL) pair
NAU7802                     — 24-bit load cell ADC driver
ICM20948                    — 9-axis IMU driver (bank-switched registers)
SensorReader (ABC)
├── SuspensionReader        — AIN0–3, NAU7802×4, ICM-20948
├── DrivetrainReader        — NAU7802×1, Hall RPM, stepper USER_RAM write
└── SteeringReader          — NAU7802×1, rotary encoder
AcquisitionThread           — threading.Thread, owns ChannelFilter instances
CSVLogger                   — writes acquisition-filtered DataFrames to CSV
```

### AcquisitionThread Constructor (current signature)
```python
AcquisitionThread(
    mode:             TestMode,
    active_channels:  List[ChannelConfig],
    sample_rate_hz:   float,
    buffer:           RingBuffer,
    status_callback:  Callable[[AcqStatus, str], None],
    ip_address:       str           = T7_IP_ADDRESS,
    connection_type:  str           = T7_CONNECTION,
    filter_profile:   FilterProfile = FilterProfile.NONE,
    filter_n:         int           = 5,
    filter_ema_alpha: float         = 0.25,
)
```

---

## dashboard.py — Current Class Structure

```
ChannelValueCard (QFrame)
│   ├── name label + filter QComboBox + Zero QPushButton  (top row)
│   ├── value label + unit label                          (bottom row)
│   ├── _display_filter: ChannelFilter                    (display-layer filter)
│   ├── _display_offset: float                            (Zero button offset)
│   └── update_value(acq_filtered_value) → float         (returns display value)
│
MultiChannelPlot (QWidget)   — pyqtgraph time series, rolling/absolute X mode
ChannelSelectorPanel (QWidget) — scrollable checkbox list, All/None buttons
CrossPlotWidget (QWidget)    — X vs Y scatter plot with channel selectors
MainWindow (QMainWindow)
    ├── Sidebar: theme, mode, connection, sample rate,
    │           Acq Filter (CSV) group [Profile combo + N spinbox],
    │           acquisition controls, CSV recording, channel selector
    ├── Content: value cards row + tabbed plots (Time Series / Cross Plot)
    └── _acq_filter_profile: FilterProfile   ← set by sidebar combo
        _acq_filter_n: int                   ← set by sidebar N spinbox
```

### ChannelValueCard Filter Options (current)
```python
_FILTER_OPTIONS = [
    ("None",   FilterProfile.NONE, 5),
    ("SMA 5",  FilterProfile.SMA,  5),
    ("SMA 10", FilterProfile.SMA,  10),
]
```
To add `Median+EMA` when ready: append `("Median+EMA", FilterProfile.MEDIAN_EMA, 5)`
to both `_FILTER_OPTIONS` (card) and `_filt_options` (sidebar).

### _refresh_plots data flow
```python
# For each frame drained from ring buffer:
display_values = {}
for key, card in self._value_cards.items():
    if key in frame.values:
        display_values[key] = card.update_value(frame.values[key])
        # update_value(): acq_value → display filter → − zero_offset → label → return

plot_frame = DataFrame(values={**frame.values, **display_values})
self._multi_plot.push_frame(plot_frame)    # display-filtered values
self._cross_plot.push_frame(plot_frame)   # display-filtered values
self._csv_logger.write_frame(frame)       # ORIGINAL acq-filtered values only
```

---

## config.py Reference

### SensorTag Enum (current)
```python
class SensorTag(Enum):
    AIN0=0, AIN1=1, AIN2=2, AIN3=3          # AIN pin number
    NAU7802_BUS0=10, BUS1=11, BUS2=12, BUS3=13
    ICM_ACCEL_X=20, Y=21, Z=22
    ICM_GYRO_X=23,  Y=24, Z=25
    HALL_RPM=30
    ENC_COUNTS=31, ENC_ANGLE=32, ENC_VELOCITY=33
    TIMESTAMP=99
```

### ChannelConfig Dataclass
```python
@dataclass
class ChannelConfig:
    key:      str               # DataFrame.values key, CSV column header
    label:    str               # UI display name
    register: int               # SensorTag.value
    unit:     str
    scale:    float = 1.0       # physical = raw * scale + offset
    offset:   float = 0.0
    y_range:  Optional[Tuple[float,float]] = None
    enabled_by_default: bool = True
    color:    str  = ""
    is_timestamp: bool = False
```

### Key Hardware Constants
```python
T7_IP_ADDRESS      = "192.168.1.10"
NAU7802_ADDR       = 0x2A           # fixed per datasheet
NAU7802_GAIN       = 128            # ADJUST to load cell spec
NAU7802_SAMPLE_RATE = 80            # SPS
ICM_ADDR           = 0x69           # AD0 HIGH; 0x68 if AD0 to GND
RPM_PIN            = 0              # FIO0
RPM_PULSES_PER_REV = 1              # ADJUST to trigger wheel tooth count
ENC_CH_A_PIN       = 2              # FIO2 (must be even)
ENC_CH_B_PIN       = 3              # FIO3
ENC_PPR            = 600            # ADJUST once encoder model confirmed
STEPPER_RPM_RAM_NAME = "USER_RAM0_F32"
WARMUP_SECONDS     = 6
```

---

## Debugging Runbook

```
Symptom: Engineer panel shows "✗ Failed — check console" for IMU channels
  → FIXED in v2.6.2: ICM accel/gyro ChannelConfig blocks lacked explicit
    offset= keyword — patcher requires both scale= and offset= to be present.
    Config.py now has offset=0.0 on all ICM blocks. If new channels are added
    without explicit offset=, the patcher now auto-inserts offset= after scale=.

Symptom: Engineer panel shows "✗ Failed" for channels with expression-valued scale
  → FIXED in v2.6.2: old regex r'(scale\s*=\s*)([+-]?\d+\.?\d*(?:[eE][+-]?\d+)?)'
    only matched numeric literals. Expression values like `1.0 / 8192.0` or
    `1.0 / 65.5` caused a silent regex miss → patcher returned False without
    writing the file. New regex r'(scale\s*=\s*)([^,\n#)]+)' matches any RHS
    up to a comma, newline, comment, or closing paren.
    NOTE: After applying a new scale to an expression-valued channel, the scale
    line will be rewritten as a plain float (e.g. 0.000244140625), not the
    original expression. This is correct and expected.

Symptom: Engineer panel shows "✓ Saved" but values revert on dashboard restart
  → patch_config_channel failed silently (returned False but status label
    showed success due to a prior bug). Verify console output for [patch_config]
    messages. If the .bak file exists but config.py is unchanged, the os.replace
    call failed — check filesystem permissions on the project directory.

Symptom: RPM reads 0 consistently despite confirmed signal on FIO0
  → MOST LIKELY: DIO#_EF_INDEX not set (mode written to ENABLE instead)
  → FIX: Use eWriteName(h, "DIO0_EF_INDEX", 3) — see Critical Rule §3
  → Also check: ticks==0 from READ_A → no edges in window → shaft stopped?
  → READ_A returns TICKS not Hz — divide 80_000_000 by ticks for freq

Symptom: Instant disconnect (< 1 s after Start)
  → Tailscale running? → always use direct IP 192.168.1.10
  → ping 192.168.1.10 from host

Symptom: Connects, disconnects after ~6 s
  → Exception in init_hardware() — check console/stderr
  → I2C NACK (LJM error 1230) = device not responding
  → Check I2C address, power, wiring

Symptom: Streams but all values zero
  → ChannelConfig.key mismatch with DataFrame.values key
  → SensorTag value mismatch — run SensorTag invariant check (Rule §5)

Symptom: LJM error 1294 (LJME_INVALID_NAME)
  → Indexed I2C register name used: "I2C_DATA_TX0"
  → FIX: eWriteNameByteArray("I2C_DATA_TX", ...) — see Rule §2

Symptom: Lua stepper not moving
  → Script too large? awk 'NF && !/^--/' t7_drivetrain_control.lua | wc -c
  → USER_RAM0_F32 being written by Python? Check STEPPER_RPM_RAM_NAME
  → EN logic is inverted on STP-DRV-4830 (EN asserted = disabled)

Symptom: Load cell values noisy
  → Check acquisition filter setting — sidebar "Acq Filter" combo
  → SMA N=5 at 10 Hz = 500 ms window; increase N for smoother at cost of lag
  → Per-card display filter adds additional smoothing without affecting CSV
```

---

## T7 Pin Assignment Map

| T7 Pin | DIO# | Mode | Signal | Notes |
|---|---|---|---|---|
| FIO0 | 0 | DIO-EF Index 3 | Hall RPM in | Active-low, no pull-up, 3.3V supply |
| FIO2 | 2 | DIO-EF Index 10 | Encoder Ch A | Steering mode, must be even pin |
| FIO3 | 3 | DIO-EF Index 10 | Encoder Ch B | Auto-paired with Ch A |
| FIO4 | 4 | DIO-EF PWM | Stepper STEP | Drivetrain mode |
| FIO5 | 5 | Digital out | Stepper DIR | Drivetrain mode |
| EIO0 | 8 | I2C SDA | NAU7802 Bus0 | LC0 Front-Left |
| EIO1 | 9 | I2C SCL | NAU7802 Bus0 | LC0 Front-Left |
| EIO2 | 10 | I2C SDA | NAU7802 Bus1 | LC1 Front-Right |
| EIO3 | 11 | I2C SCL | NAU7802 Bus1 | LC1 Front-Right |
| EIO4 | 12 | I2C SDA | NAU7802 Bus2 | LC2 Rear-Left |
| EIO5 | 13 | I2C SCL | NAU7802 Bus2 | LC2 Rear-Left |
| EIO6 | 14 | I2C SDA | NAU7802 Bus3 | LC3 Rear-Right |
| EIO7 | 15 | I2C SCL | NAU7802 Bus3 | LC3 Rear-Right |
| CIO0 | 16 | I2C SDA | ICM-20948 | IMU |
| CIO1 | 17 | I2C SCL | ICM-20948 | IMU |
| CIO2 | 18 | I2C SDA | Rotary Enc | Steering mode |
| CIO3 | 19 | I2C SCL | Rotary Enc | Steering mode |
| AIN0–3 | — | AIN | Linear Pots FL/FR/RL/RR | Suspension mode |

---

## USER_RAM Register Map (Drivetrain Mode)

| Register | Modbus | Signal | Notes |
|---|---|---|---|
| USER_RAM0_F32 | 46000 | RPM handoff | Python writes RPM → Lua reads |
| USER_RAM1_F32 | 46002 | Hall RPM | Computed RPM |
| USER_RAM2_F32 | 46004 | Stepper position | Cumulative steps |
| USER_RAM3_F32 | 46006 | Active RPM setpoint | Steady-state or ramp |
| USER_RAM4_F32 | 46008 | RPM error | measured − setpoint |
| USER_RAM5_F32 | 46010 | Control mode | 0=STEADY_STATE, 1=RAMP |
| USER_RAM6_F32 | 46012 | Ramp progress | 0.0–1.0 |
| USER_RAM7_F32 | 46014 | Timestamp | ms since script start |

---

## Adding a New Sensor

1. Add `SensorTag` entry in `config.py` with unique integer value
2. Add hardware constants (pins, address) to `config.py`
3. Add `ChannelConfig` entry to the relevant mode list
4. Add driver class or extend existing one in `acquisition.py`
5. Add read logic in `SensorReader.read_frame()` for that mode
6. Validate: `python -c "from config import *; from acquisition import *"`
7. Run SensorTag invariant check (Critical Rule §5)
8. **Update this skill file** (Self-Update Protocol above)

---

## Calibration Notes

All `ChannelConfig.scale` / `offset` default to `1.0` / `0.0` — raw counts
until calibrated. Procedure:
1. Apply known reference load/displacement
2. Read raw counts from dashboard value card
3. `scale = known_value / raw_counts`
4. Enter values in Engineer Control Panel → Apply → values write to config.py
5. Restart stream — no code change needed

**Engineer panel patcher rules (v2.6.2+):**
- Both `scale=` and `offset=` must appear as explicit kwargs in the ChannelConfig
  block. All blocks in config.py now have both. If you add a new channel, always
  include both explicitly — do not rely on dataclass defaults.
- Expression-valued scales (`1.0 / 8192.0`) are replaced with their evaluated
  float on first Apply. This is correct — the patcher stores the computed value.
- A `.bak` backup of config.py is created before every write.

Load cells: raw 24-bit ADC counts → calibrate to N or N·m
IMU: raw 16-bit counts → scale factors in ICM-20948 datasheet
Potentiometers: AIN voltage (V) → calibrate to mm of travel
