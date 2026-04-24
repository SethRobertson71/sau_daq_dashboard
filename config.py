# ==============================================================================
# config.py — Channel Definitions, Hardware Descriptors, Application Config
# Vehicle Sensor Suite Dashboard  v2.0.0
# ==============================================================================
#
# ARCHITECTURE CHANGE (v1 → v2):
#   Sensor I2C reads now happen entirely in Python via the LJM library over
#   Ethernet. T7 Lua scripts are no longer used for sensor acquisition.
#   ChannelConfig.register stores a SensorTag enum value that tells
#   acquisition.py which hardware device/channel to read for each channel.
#
#   The T7 Lua layer is retained only for the drivetrain stepper control loop
#   (t7_drivetrain_control.lua), which runs independently on the T7 and reads
#   RPM from USER_RAM0_F32 written by this Python layer after each read.
#
# ==============================================================================

from __future__ import annotations
import os
from dataclasses import dataclass
from enum import Enum
from typing import List, Dict, Optional, Tuple


# ==============================================================================
# TEST MODE
# ==============================================================================
class TestMode(Enum):
    SUSPENSION = "Suspension Testing"
    DRIVETRAIN = "Drivetrain Testing"
    STEERING   = "Steering Testing"


# ==============================================================================
# SENSOR SOURCE TAGS
# Each ChannelConfig.register holds one of these integer values.
# acquisition.py uses the tag to determine how to read the channel.
# ==============================================================================
class SensorTag(Enum):
    # Analog inputs — read via AIN# LJM name
    AIN0 = 0
    AIN1 = 1
    AIN2 = 2
    AIN3 = 3

    # NAU7802 load cell amplifiers — each on a dedicated soft I2C bus.
    # Tag value encodes which bus index (0–3 matching NAU7802_BUSES dict below).
    NAU7802_BUS0 = 10   # EIO0/1  Front Left
    NAU7802_BUS1 = 11   # EIO2/3  Front Right
    NAU7802_BUS2 = 12   # EIO4/5  Back Left
    NAU7802_BUS3 = 13   # EIO6/7  Back Right

    # ICM-20948 IMU axes — all six read in one burst, returned individually
    ICM_ACCEL_X = 20
    ICM_ACCEL_Y = 21
    ICM_ACCEL_Z = 22
    ICM_GYRO_X  = 23
    ICM_GYRO_Y  = 24
    ICM_GYRO_Z  = 25

    # DIO Extended Features
    HALL_RPM     = 30   # FIO0  DIO-EF Mode 3 frequency → RPM
    ENC_COUNTS   = 31   # FIO2  DIO-EF Mode 10 quadrature raw counts
    ENC_ANGLE    = 32   # Derived from ENC_COUNTS (counts / ENC_COUNTS_PER_DEG)
    ENC_VELOCITY = 33   # Derived first-difference (deg/s)

    # Software-derived
    TIMESTAMP = 99      # ms elapsed since acquisition start


# ==============================================================================
# CHANNEL CONFIGURATION
# ==============================================================================
@dataclass
class ChannelConfig:
    key: str            # Internal ID and CSV column header
    label: str          # UI display name
    register: int       # SensorTag.value — tells acquisition.py what to read
    unit: str           # Physical unit string for axis labels

    # physical_value = (raw_value * scale) + offset
    # ADJUST: Fill in calibration values once calibration project is complete.
    scale: float  = 1.0
    offset: float = 0.0

    enabled_by_default: bool = True
    y_range: Optional[Tuple[float, float]] = None
    color: str = ""
    is_timestamp: bool = False
    # Default filter shown in the per-card display dropdown on startup.
    # "none", "sma5", "sma10", or "med_ema"
    default_display_filter: str = "none"


# ==============================================================================
# COLOR CYCLES — assigned to channels in order
# Two palettes: saturated neons for dark backgrounds, muted tones for light.
# dashboard.py selects between them based on the active theme.
# ==============================================================================

# Dark theme — high-saturation, high-contrast against dark backgrounds
COLOR_CYCLE = [
    "#00D4FF", "#FF6B35", "#7AFF6B", "#FFD700",
    "#FF4FA3", "#B47FFF", "#FF4444", "#4FFFFF",
    "#FFAA00", "#88FF88", "#FF88FF", "#AAAAFF",
    "#FF8800", "#00FFAA",
]

# Light theme — muted, professional tones legible on light gray backgrounds.
# Desaturated and darkened versions of the dark palette hues so channel
# identity is preserved but the colors don't bleed on a light background.
COLOR_CYCLE_LIGHT = [
    "#0077AA",   # steel blue      (was cyan #00D4FF)
    "#C94F00",   # burnt orange    (was #FF6B35)
    "#3A8A3A",   # forest green    (was lime #7AFF6B)
    "#997700",   # dark gold       (was #FFD700)
    "#A0266A",   # muted magenta   (was #FF4FA3)
    "#6040A0",   # slate violet    (was #B47FFF)
    "#B52222",   # brick red       (was #FF4444)
    "#007A8A",   # teal            (was #4FFFFF)
    "#A06000",   # dark amber      (was #FFAA00)
    "#2E7D32",   # deep green      (was #88FF88)
    "#7B3F7B",   # dusty purple    (was #FF88FF)
    "#3A5080",   # navy            (was #AAAAFF)
    "#A05000",   # sienna          (was #FF8800)
    "#006655",   # dark teal green (was #00FFAA)
]


# ==============================================================================
# T7 HARDWARE CONFIGURATION
# All I2C pin assignments, addresses, and sensor parameters live here.
# acquisition.py reads these — adjust here, not in acquisition.py.
# ==============================================================================

# --- T7 Connection ------------------------------------------------------------
# ADJUST: Set to your T7's static Ethernet IP address.
T7_IP_ADDRESS = "192.168.1.10"    # ADJUST: set to your T7 static IP
T7_CONNECTION = "Ethernet"   # "Ethernet" or "USB"

# --- I2C Bus ------------------------------------------------------------------
# I2C_SPEED_THROTTLE: 65516 ≈ 100 kHz per LabJack formula.
# Formula: 65536 - (80_000_000 / (2 * desired_hz))
# ADJUST: Use 64736 for ~50 kHz if signal integrity issues appear.
I2C_SPEED_THROTTLE = 65516
I2C_OPTIONS        = 0

# NAU7802 soft I2C bus pin pairs {bus_index: (SDA_pin, SCL_pin)}
# T7 DIO numbering: FIO0–7=0–7, EIO0–7=8–15, CIO0–3=16–19
# ADJUST: Update if wiring changes.
NAU7802_BUSES: Dict[int, Tuple[int, int]] = {
    0: (8,  9 ),   # EIO0/1  Load Cell 0 Front Left
    1: (10, 11),   # EIO2/3  Load Cell 1 Front Right
    2: (12, 13),   # EIO4/5  Load Cell 2 Back Left
    3: (14, 15),   # EIO6/7  Load Cell 3 Back Right
}

# NAU7802 I2C address — fixed 0x2A per datasheet §6.1.
# All 4 share this address; bus isolation provides uniqueness.
# ADJUST: Only if replacing with an ADC that has configurable addressing.
NAU7802_ADDR = 0x2A

# NAU7802 PGA gain. Valid: 1,2,4,8,16,32,64,128
# ADJUST: Match to load cell full-scale output (mV/V) and bridge excitation.
NAU7802_GAIN = 128

# NAU7802 output data rate (SPS). Valid: 10,20,40,80,320
# 10 SPS = best noise performance. 80 SPS = faster response.
# ADJUST: 10 SPS strongly recommended at 10 Hz acquisition rate.
NAU7802_SAMPLE_RATE = 80

# NAU7802 LDO voltage bits for CTRL1. 0x18 = 3.0V.
# ADJUST: Match to load cell bridge excitation voltage requirement.
# Encoding: 000=2.4V 001=2.7V 010=3.0V 011=3.3V 100=3.6V 101=3.9V 110=4.2V 111=4.5V
NAU7802_LDO_BITS = 0x18

# ICM-20948 I2C bus pins (CIO0=DIO16, CIO1=DIO17) and address
ICM_SDA  = 16       # CIO0
ICM_SCL  = 17       # CIO1
# AD0 HIGH (3.3V) → 0x69. ADJUST: Change to 0x68 if AD0 rewired to GND.
ICM_ADDR = 0x69

# ICM-20948 full-scale ranges
# ADJUST during calibration. Also update ICM_ACCEL_SCALE / ICM_GYRO_SCALE.
# Accel valid: 2, 4, 8, 16 (g)
ICM_ACCEL_RANGE_G  = 4
# Gyro valid: 250, 500, 1000, 2000 (dps)
ICM_GYRO_RANGE_DPS = 500

# Scale factors — must match range values above.
# Accel: ±2g=16384, ±4g=8192, ±8g=4096, ±16g=2048 LSB/g
# Gyro:  ±250=131.0, ±500=65.5, ±1000=32.8, ±2000=16.4 LSB/dps
ICM_ACCEL_SCALE = 8192.0
ICM_GYRO_SCALE  = 65.5

# --- DIO Extended Features ----------------------------------------------------
# Hall effect RPM — FIO0, DIO-EF Mode 3 (frequency input)
# !! HARDWARE ACTION ITEM: 3.3 kΩ pull-up to 3.3V required on FIO0 !!
RPM_PIN            = 0
RPM_PULSES_PER_REV = 1        # ADJUST: set to trigger wheel tooth count
RPM_WINDOW_US      = 1000000  # µs measurement window (ADJUST: 100000 for 100ms)

# Rotary encoder — FIO2/FIO3, DIO-EF Mode 10 (quadrature)
ENC_CH_A_PIN       = 2        # FIO2 (must be even)
ENC_CH_B_PIN       = 3        # FIO3 (must be Ch A + 1)
ENC_PPR            = 600      # ADJUST: set to encoder spec once model confirmed
ENC_QUAD_MUL       = 4        # Always 4 for full quadrature — do not change
ENC_COUNTS_PER_DEG = (ENC_PPR * ENC_QUAD_MUL) / 360.0

# --- Analog Inputs ------------------------------------------------------------
# Linear potentiometers — 0–10V DC on AIN0–3.
# ADJUST: Change to 5.0 if supply voltage changes to 0–5V.
AIN_RANGE_VOLTS      = 10.0
AIN_RESOLUTION_INDEX = 0   # 0=16-bit default (ADJUST: 1-2 for better SNR)

# --- Stepper control USER_RAM handoff ----------------------------------------
# Python writes RPM to this register after each read.
# t7_drivetrain_control.lua reads it to drive the stepper control loop.
# ADJUST: Must match the register read in t7_drivetrain_control.lua.
STEPPER_RPM_RAM_NAME = "USER_RAM0_F32"   # LJM register name


# ==============================================================================
# APPLICATION CONSTANTS
# ==============================================================================
SAMPLE_RATE_HZ_DEFAULT = 10
SAMPLE_RATE_HZ_MIN     = 1
SAMPLE_RATE_HZ_MAX     = 50    # Practical ceiling with full I2C channel set
PLOT_HISTORY_SECONDS   = 30
BUFFER_MAXLEN          = 10000
PLOT_REFRESH_MS        = 50    # 20 FPS
WARMUP_SECONDS         = 6     # 5s sensor warm-up + 1s margin

# --- Filter defaults ---------------------------------------------------------
# FILTER_EMA_ALPHA  : first EMA stage in Median+EMA cascade (broadband smooth).
#                     fc ≈ −fs × ln(1−α) / 2π
#                     At 10 Hz, 0.25 → fc ≈ 0.45 Hz
# FILTER_EMA2_ALPHA : second EMA stage (additional low-pass attenuation).
#                     At 10 Hz, 0.10 → fc ≈ 0.17 Hz (~16 dB at 1 Hz)
#                     ADJUST: 0.15 for moderate, 0.05 for aggressive.
FILTER_EMA_ALPHA  = 0.25   # first EMA stage
FILTER_EMA2_ALPHA = 0.10   # second EMA stage — ADJUST to taste

CSV_DEFAULT_DIR = os.path.join(
    os.path.expanduser("~"), "Documents", "VehicleTestData"
)

APP_TITLE   = "Vehicle Sensor Suite"
APP_VERSION = "2.5.1"


# ==============================================================================
# CHANNEL DEFINITIONS
# ==============================================================================

SUSPENSION_CHANNELS: List[ChannelConfig] = [
    ChannelConfig(
        key="timestamp_ms", label="Timestamp",
        register=SensorTag.TIMESTAMP.value, unit="ms",
        is_timestamp=True,
    ),
    ChannelConfig(
        key="linpot_fl", label="Travel FL",
        register=SensorTag.AIN0.value, unit="mm",
        scale=15.45, offset=0.015,    # ADJUST: mm/V calibration factor
        y_range=(0, 153),
    ),
    ChannelConfig(
        key="linpot_fr", label="Travel FR",
        register=SensorTag.AIN1.value, unit="V",
        scale=1.0, offset=0.0,
        y_range=(0, 10),
    ),
    ChannelConfig(
        key="linpot_bl", label="Travel BL",
        register=SensorTag.AIN2.value, unit="V",
        scale=1.0, offset=0.0,
        y_range=(0, 10),
    ),
    ChannelConfig(
        key="linpot_rr", label="Travel RR",
        register=SensorTag.AIN3.value, unit="V",
        scale=1.0, offset=0.0,
        y_range=(0, 10),
    ),
    ChannelConfig(
        key="loadcell_fl", label="Load Cell FL",
        register=SensorTag.NAU7802_BUS0.value, unit="counts",
        scale=1.0, offset=0.0,    # ADJUST: N/count after calibration
        default_display_filter="med_ema",
    ),
    ChannelConfig(
        key="loadcell_fr", label="Load Cell FR",
        register=SensorTag.NAU7802_BUS1.value, unit="lbs",
        scale=1.0,      # change this
        offset=0.0,     # change this
        default_display_filter="med_ema",
    ),
    ChannelConfig(
        key="loadcell_bl", label="Load Cell BL",
        register=SensorTag.NAU7802_BUS2.value, unit="counts",
        scale=1.0, offset=0.0,
        default_display_filter="med_ema",
    ),
    ChannelConfig(
        key="loadcell_br", label="Load Cell BR",
        register=SensorTag.NAU7802_BUS3.value, unit="counts",
        scale=1.0, offset=0.0,
        default_display_filter="med_ema",
    ),
    ChannelConfig(
        key="imu_accel_x", label="Accel X",
        register=SensorTag.ICM_ACCEL_X.value, unit="g",
        scale=1.0 / 8192.0,       # ADJUST divisor if ICM_ACCEL_RANGE_G changes
        y_range=(-4, 4),
    ),
    ChannelConfig(
        key="imu_accel_y", label="Accel Y",
        register=SensorTag.ICM_ACCEL_Y.value, unit="g",
        scale=1.0 / 8192.0,
        y_range=(-4, 4),
    ),
    ChannelConfig(
        key="imu_accel_z", label="Accel Z",
        register=SensorTag.ICM_ACCEL_Z.value, unit="g",
        scale=1.0 / 8192.0,
        y_range=(-4, 4),
    ),
    ChannelConfig(
        key="imu_gyro_x", label="Gyro X",
        register=SensorTag.ICM_GYRO_X.value, unit="dps",
        scale=1.0 / 65.5,         # ADJUST divisor if ICM_GYRO_RANGE_DPS changes
        y_range=(-500, 500), enabled_by_default=False,
    ),
    ChannelConfig(
        key="imu_gyro_y", label="Gyro Y",
        register=SensorTag.ICM_GYRO_Y.value, unit="dps",
        scale=1.0 / 65.5,
        y_range=(-500, 500), enabled_by_default=False,
    ),
    ChannelConfig(
        key="imu_gyro_z", label="Gyro Z",
        register=SensorTag.ICM_GYRO_Z.value, unit="dps",
        scale=1.0 / 65.5,
        y_range=(-500, 500), enabled_by_default=False,
    ),
]

DRIVETRAIN_CHANNELS: List[ChannelConfig] = [
    ChannelConfig(
        key="timestamp_ms", label="Timestamp",
        register=SensorTag.TIMESTAMP.value, unit="ms",
        is_timestamp=True,
    ),
    ChannelConfig(
        key="loadcell_fl", label="Load Cell FL",
        register=SensorTag.NAU7802_BUS0.value, unit="counts",
        scale=1.0, offset=0.0,    # ADJUST: N·m/count after calibration
    ),
    ChannelConfig(
        key="rpm", label="Crankshaft RPM",
        register=SensorTag.HALL_RPM.value, unit="RPM",
        scale=1.0, offset=0.0,
        y_range=(0, 8000),        # ADJUST: set to engine/motor max RPM
    ),
]

STEERING_CHANNELS: List[ChannelConfig] = [
    ChannelConfig(
        key="timestamp_ms", label="Timestamp",
        register=SensorTag.TIMESTAMP.value, unit="ms",
        is_timestamp=True,
    ),
    ChannelConfig(
        key="loadcell_torque", label="Steering Torque",
        register=SensorTag.NAU7802_BUS0.value, unit="counts",
        scale=1.0, offset=0.0,    # ADJUST: N·m/count after calibration
    ),
    ChannelConfig(
        key="encoder_counts", label="Encoder Counts",
        register=SensorTag.ENC_COUNTS.value, unit="counts",
        enabled_by_default=False,
    ),
    ChannelConfig(
        key="encoder_angle", label="Steering Angle",
        register=SensorTag.ENC_ANGLE.value, unit="deg",
        scale=1.0, offset=0.0,
        y_range=(-720, 720),      # ADJUST: actual steering lock-to-lock range
    ),
    ChannelConfig(
        key="encoder_velocity", label="Steering Rate",
        register=SensorTag.ENC_VELOCITY.value, unit="deg/s",
    ),
]

MODE_CHANNELS: Dict[TestMode, List[ChannelConfig]] = {
    TestMode.SUSPENSION: SUSPENSION_CHANNELS,
    TestMode.DRIVETRAIN: DRIVETRAIN_CHANNELS,
    TestMode.STEERING:   STEERING_CHANNELS,
}
