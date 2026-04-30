# ==============================================================================
# acquisition.py — Host-Side LJM Sensor Acquisition Backend
# Vehicle Sensor Suite Dashboard  v2.0.0
# ==============================================================================
#
# ARCHITECTURE (v2):
#   All sensor I2C transactions are performed by this module over Ethernet
#   using the LabJack LJM Python library. No T7 Lua scripts are needed for
#   sensor acquisition.
#
# INTERFACE TO dashboard.py (UNCHANGED FROM v1):
#   AcquisitionThread — same constructor, same start/stop interface
#   RingBuffer        — same push/drain API
#   CSVLogger         — same open/write_frame/close API
#   DataFrame         — same dataclass
#   AcqStatus         — same enum
#
# SENSOR READ ARCHITECTURE:
#   SensorReader base class + mode-specific subclasses handle all hardware.
#   Each subclass implements:
#     init_hardware()  — configure AIN ranges, DIO-EF modes, I2C device init
#     read_frame()     — read all active channels, return dict {key: value}
#
# I2C TRANSACTION BATCHING:
#   eWriteNames() batches all configuration writes + I2C_GO into one Ethernet
#   packet per transaction. This is the key performance optimization —
#   5–8 register writes become one round trip instead of 5–8 separate calls.
#
# ==============================================================================

from __future__ import annotations

import threading
import time
import datetime
import collections
import struct
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import List, Dict, Optional, Callable, Tuple

from config import (
    TestMode, ChannelConfig, SensorTag, MODE_CHANNELS,
    T7_IP_ADDRESS, T7_CONNECTION,
    I2C_SPEED_THROTTLE, I2C_OPTIONS,
    NAU7802_BUSES, NAU7802_ADDR,
    NAU7802_GAIN, NAU7802_SAMPLE_RATE, NAU7802_LDO_BITS,
    ICM_SDA, ICM_SCL, ICM_ADDR,
    ICM_ACCEL_RANGE_G, ICM_GYRO_RANGE_DPS,
    ICM_ACCEL_SCALE, ICM_GYRO_SCALE,
    RPM_PIN, RPM_PULSES_PER_REV, RPM_WINDOW_US,
    ENC_CH_A_PIN, ENC_CH_B_PIN, ENC_COUNTS_PER_DEG,
    AIN_RANGE_VOLTS, AIN_RESOLUTION_INDEX,
    STEPPER_RPM_RAM_NAME,
    BUFFER_MAXLEN, WARMUP_SECONDS,
    FILTER_EMA_ALPHA, FILTER_EMA2_ALPHA,
)


# ==============================================================================
# FILTER PROFILES
# ==============================================================================

class FilterProfile(Enum):
    """
    Named filter profiles selectable per channel.

    NONE       — raw sensor values, no smoothing.
    SMA        — Simple Moving Average over the last N samples.
                 Uniform rectangular window.
    MEDIAN_EMA — Cascaded Median spike-reject → EMA broadband smooth.
                 Stage 1: running median over the last N samples.
                   Eliminates impulse noise (I2C outliers, mechanical
                   shock spikes) without smearing them into adjacent
                   samples the way a mean-based filter would.
                 Stage 2: EMA on the median output.
                   Smooths residual Gaussian ADC noise. alpha controls
                   the time constant — higher = faster, less smooth.
                 At 10 Hz with N=5, alpha=0.25:
                   Median window  = 500 ms  (spike must persist > 2 samples
                                             to pass through)
                   EMA time const ≈ 350 ms  (−3 dB at ~0.45 Hz)
    """
    NONE       = "None"
    SMA        = "SMA"
    MEDIAN_EMA = "Median+EMA"


class ChannelFilter:
    """
    Per-channel stateful filter.  One instance per active channel, owned by
    AcquisitionThread._acquire_loop() and by ChannelValueCard (display layer).

    All filter state is internal — call push(raw) → returns filtered float.
    Call reset() to clear state (e.g. on stream restart).

    ACQUISITION LAYER  — filters the value before it enters the ring buffer.
                         CSV logs the filtered value.
    DISPLAY LAYER      — ChannelValueCard holds a second independent instance.
                         Applies a lighter additional smoothing pass for visual
                         presentation only.  Does not affect CSV data.

    Parameters
    ----------
    profile : FilterProfile
        Which algorithm to run.
    sma_n : int
        Window length for SMA and for the median stage of MEDIAN_EMA.
        Must be odd for a clean median (enforced internally — incremented
        by 1 if even so the middle index is unambiguous).
        Effective latency at 10 Hz: sma_n * 100 ms.
        Recommended: 5 (500 ms) for load cells at 5–10 Hz.
    ema_alpha : float
        First EMA stage smoothing coefficient (0 < α < 1).
        Time constant: τ ≈ −1 / (fs × ln(1 − α)).
        At 10 Hz, α=0.25 → fc ≈ 0.45 Hz.
        Recommended: 0.2–0.3 at 10 Hz.
    ema2_alpha : float
        Second EMA stage smoothing coefficient (0 < α < 1).
        Applied after the first EMA in MEDIAN_EMA cascade.
        Lower = stronger low-pass attenuation, slower step response.
        At 10 Hz: 0.15 → fc≈0.26 Hz (~10 dB at 1 Hz)
                  0.10 → fc≈0.17 Hz (~16 dB at 1 Hz)
                  0.05 → fc≈0.08 Hz (~22 dB at 1 Hz)
        Default: FILTER_EMA2_ALPHA from config.py (0.10).
        Set to 1.0 to disable the second stage (passthrough).
    """

    def __init__(
        self,
        profile:    FilterProfile = FilterProfile.NONE,
        sma_n:      int   = 5,
        ema_alpha:  float = FILTER_EMA_ALPHA,
        ema2_alpha: float = FILTER_EMA2_ALPHA,
    ):
        self.profile    = profile
        # Force odd window length for unambiguous median centre index
        n = max(3, int(sma_n))
        self.sma_n      = n if n % 2 == 1 else n + 1
        self.ema_alpha  = max(0.01, min(0.99, float(ema_alpha)))
        self.ema2_alpha = max(0.01, min(1.00, float(ema2_alpha)))

        # Shared sample window — capped deque, O(1) append
        self._window: collections.deque = collections.deque(maxlen=self.sma_n)
        # First EMA state
        self._ema_value:  Optional[float] = None
        # Second EMA state
        self._ema2_value: Optional[float] = None

    # ------------------------------------------------------------------
    def push(self, raw: float) -> float:
        """Feed one raw sample, return the filtered output."""
        if self.profile == FilterProfile.NONE:
            return raw

        if self.profile == FilterProfile.SMA:
            return self._sma(raw)

        if self.profile == FilterProfile.MEDIAN_EMA:
            return self._median_ema(raw)

        return raw  # fallthrough safety

    def reset(self):
        """Clear all filter state — call on stream start/stop."""
        self._window.clear()
        self._ema_value  = None
        self._ema2_value = None

    # ------------------------------------------------------------------
    # Internal algorithm implementations
    # ------------------------------------------------------------------

    def _sma(self, raw: float) -> float:
        """
        Simple Moving Average over the last sma_n samples.
        Returns the window mean from sample 1 (no zero-padding), so there
        is no downward step artifact during the initial window fill.
        """
        self._window.append(raw)
        return sum(self._window) / len(self._window)

    def _median(self) -> float:
        """
        Running median of the current window contents.
        Window is a deque of the last sma_n samples.  Sorting a small
        list (N ≤ 11 recommended) is faster in practice than maintaining
        a sorted structure due to Python list overhead at these sizes.
        The middle index is unambiguous because sma_n is forced odd.
        """
        w = sorted(self._window)
        return w[len(w) // 2]

    def _ema(self, value: float) -> float:
        """
        First-stage Exponential Moving Average.
        Seeds from the first input value to avoid a startup transient.
        """
        if self._ema_value is None:
            self._ema_value = value
        else:
            self._ema_value = (
                self.ema_alpha * value
                + (1.0 - self.ema_alpha) * self._ema_value
            )
        return self._ema_value

    def _ema2(self, value: float) -> float:
        """
        Second-stage Exponential Moving Average (additional low-pass).
        Seeds from the first input value independently of the first stage.
        Set ema2_alpha = 1.0 to bypass (output = input, no attenuation).
        """
        if self._ema2_value is None:
            self._ema2_value = value
        else:
            self._ema2_value = (
                self.ema2_alpha * value
                + (1.0 - self.ema2_alpha) * self._ema2_value
            )
        return self._ema2_value

    def _median_ema(self, raw: float) -> float:
        """
        Cascaded Median → EMA1 → EMA2 filter.

        Stage 1 — Median (spike rejection):
          Appends raw sample to the window deque and takes the running
          median.  A single outlier is suppressed as long as it does not
          constitute more than half the window — i.e. a spike must persist
          for > sma_n // 2 consecutive samples to pass through.

        Stage 2 — EMA1 (broadband smoothing):
          First-order IIR low-pass on the median output.
          fc ≈ −fs × ln(1 − ema_alpha) / 2π
          At 10 Hz, alpha=0.25 → fc ≈ 0.45 Hz.

        Stage 3 — EMA2 (additional low-pass attenuation):
          Second first-order IIR stage cascaded after EMA1.
          Combined -3 dB frequency is lower than either stage alone.
          At 10 Hz, alpha1=0.25 + alpha2=0.10:
            EMA1 alone  → fc ≈ 0.45 Hz
            EMA2 alone  → fc ≈ 0.17 Hz
            Cascaded    → fc ≈ 0.14 Hz, ~20 dB/decade rolloff above fc
          Set ema2_alpha=1.0 in config.py to disable this stage.

        Seeds all EMA stages from their first input to avoid startup
        transients.  Partial-window median during initial fill converges
        within sma_n samples (500 ms at 10 Hz, N=5).
        """
        self._window.append(raw)
        median_val = self._median()
        ema1_val   = self._ema(median_val)
        return self._ema2(ema1_val)


# ==============================================================================
# STATUS ENUM AND DATA TYPES
# ==============================================================================

class AcqStatus(Enum):
    DISCONNECTED = "Disconnected"
    CONNECTING   = "Connecting..."
    CONNECTED    = "Connected"
    STREAMING    = "Streaming"
    ERROR        = "Error"
    STOPPED      = "Stopped"


@dataclass
class DataFrame:
    wall_time: float           # Seconds since epoch
    t7_timestamp_ms: float     # ms since acquisition start
    values: Dict[str, float]   # {channel_key: physical_value}


# ==============================================================================
# RING BUFFER
# ==============================================================================

class RingBuffer:
    """Thread-safe single-producer, single-consumer ring buffer."""

    def __init__(self, maxlen: int = BUFFER_MAXLEN):
        self._buf: collections.deque = collections.deque(maxlen=maxlen)

    def push(self, frame: DataFrame):
        self._buf.append(frame)

    def drain(self) -> List[DataFrame]:
        frames = []
        while self._buf:
            try:
                frames.append(self._buf.popleft())
            except IndexError:
                break
        return frames

    def __len__(self):
        return len(self._buf)


# ==============================================================================
# LJM I2C HELPERS
# Low-level wrappers around LJM eWriteNames / eReadNames for I2C transactions.
# All batched into as few Ethernet round trips as possible.
# ==============================================================================

class LJMi2c:
    """
    Manages soft I2C transactions on the T7 via LJM.
    Each instance is bound to one SDA/SCL pin pair (one bus).

    DATA BUFFER API NOTE:
    This LJM version does not support individually named I2C data registers
    (I2C_DATA_TX0, I2C_DATA_RX0 etc. return LJME_INVALID_NAME 1294).
    Instead, TX data is written via eWriteNameByteArray("I2C_DATA_TX", ...)
    and RX data is read via eReadNameByteArray("I2C_DATA_RX", ...).
    Control registers (I2C_NUM_BYTES_TX, I2C_NUM_BYTES_RX, I2C_GO, I2C_ACKS,
    I2C_SDA_DIONUM etc.) use standard eWriteNames / eReadName.
    """

    def __init__(self, handle, ljm, sda: int, scl: int, addr: int):
        self._handle = handle
        self._ljm    = ljm
        self._sda    = sda
        self._scl    = scl
        self._addr   = addr

    def _configure_bus(self):
        """Configure SDA, SCL, speed, options, and slave address."""
        self._ljm.eWriteNames(self._handle, 5,
            ["I2C_SDA_DIONUM", "I2C_SCL_DIONUM",
             "I2C_SPEED_THROTTLE", "I2C_OPTIONS",
             "I2C_SLAVE_ADDRESS"],
            [self._sda, self._scl,
             I2C_SPEED_THROTTLE, I2C_OPTIONS,
             self._addr]
        )

    def write_reg(self, reg: int, data: int) -> int:
        """
        Write one byte to a register.
        TX payload = [reg_address, data_byte] (2 bytes).
        Returns I2C_ACKS (0 = all bytes acknowledged).
        """
        self._configure_bus()
        # Set byte counts
        self._ljm.eWriteNames(self._handle, 2,
            ["I2C_NUM_BYTES_TX", "I2C_NUM_BYTES_RX"],
            [2, 0]
        )
        # Write TX data as byte array: [register_address, data_byte]
        self._ljm.eWriteNameByteArray(
            self._handle, "I2C_DATA_TX", 2, [reg, data]
        )
        # Trigger transaction
        self._ljm.eWriteName(self._handle, "I2C_GO", 1)
        return int(self._ljm.eReadName(self._handle, "I2C_ACKS"))

    def read_reg(self, reg: int) -> int:
        """
        Read one byte from a register.
        TX = [reg_address], RX = [data_byte].
        """
        self._configure_bus()
        self._ljm.eWriteNames(self._handle, 2,
            ["I2C_NUM_BYTES_TX", "I2C_NUM_BYTES_RX"],
            [1, 1]
        )
        self._ljm.eWriteNameByteArray(
            self._handle, "I2C_DATA_TX", 1, [reg]
        )
        self._ljm.eWriteName(self._handle, "I2C_GO", 1)
        rx = self._ljm.eReadNameByteArray(self._handle, "I2C_DATA_RX", 1)
        return int(rx[0])

    def read_burst(self, reg: int, num: int) -> List[int]:
        """
        Burst-read num bytes starting from reg in one I2C transaction.
        Returns list of ints [byte0, byte1, ..., byte_n-1].
        """
        self._configure_bus()
        self._ljm.eWriteNames(self._handle, 2,
            ["I2C_NUM_BYTES_TX", "I2C_NUM_BYTES_RX"],
            [1, num]
        )
        self._ljm.eWriteNameByteArray(
            self._handle, "I2C_DATA_TX", 1, [reg]
        )
        self._ljm.eWriteName(self._handle, "I2C_GO", 1)
        rx = self._ljm.eReadNameByteArray(self._handle, "I2C_DATA_RX", num)
        return [int(b) for b in rx]

    @staticmethod
    def to_int16(msb: int, lsb: int) -> int:
        """Signed 16-bit big-endian decode."""
        raw = (msb << 8) | lsb
        return raw - 65536 if raw >= 32768 else raw

    @staticmethod
    def to_int24(b2: int, b1: int, b0: int) -> int:
        """Signed 24-bit big-endian decode."""
        raw = (b2 << 16) | (b1 << 8) | b0
        return raw - 16777216 if raw >= 8388608 else raw


# ==============================================================================
# NAU7802 DRIVER
# ==============================================================================

class NAU7802:
    """
    Driver for one NAU7802 load cell amplifier on a dedicated I2C bus.
    Handles init, calibration, and single-read with conversion-ready polling.
    """

    # Register addresses
    REG_PU_CTRL = 0x00
    REG_CTRL1   = 0x01
    REG_CTRL2   = 0x02
    REG_ADCO_B2 = 0x12   # MSB of 24-bit ADC output (burst read × 3)

    # PU_CTRL bit masks
    RR    = 0x01   # Register Reset
    PUD   = 0x02   # Power-Up Digital
    PUA   = 0x04   # Power-Up Analog
    PUR   = 0x08   # Power-Up Ready (read-only)
    CS    = 0x10   # Cycle Start
    CR    = 0x20   # Conversion Ready (read-only)
    AVDDS = 0x80   # AVDD source select (internal LDO)

    GAIN_ENC = {1:0, 2:1, 4:2, 8:3, 16:4, 32:5, 64:6, 128:7}
    RATE_ENC = {10:0, 20:1, 40:2, 80:3, 320:7}

    def __init__(self, bus: LJMi2c, label: str):
        self._bus   = bus
        self._label = label

    def init(self) -> bool:
        """
        Full NAU7802 initialization sequence:
        reset → power-up digital → power-up analog → configure gain/rate
        → internal zero-scale calibration.
        Returns True on success.
        """
        b = self._bus

        # Register reset
        b.write_reg(self.REG_PU_CTRL, self.RR)
        time.sleep(0.002)
        b.write_reg(self.REG_PU_CTRL, 0x00)
        time.sleep(0.002)

        # Power up digital
        b.write_reg(self.REG_PU_CTRL, self.PUD)
        time.sleep(0.002)

        # Check power-up ready
        pu = b.read_reg(self.REG_PU_CTRL)
        if not (pu & self.PUR):
            print(f"NAU7802 {self._label}: WARNING — PUR not set after power-up")

        # CTRL1: LDO voltage + PGA gain
        gain_bits = self.GAIN_ENC.get(NAU7802_GAIN, 7)
        b.write_reg(self.REG_CTRL1, NAU7802_LDO_BITS | gain_bits)

        # Power up analog, enable internal LDO, start conversions
        b.write_reg(self.REG_PU_CTRL,
                    self.PUD | self.PUA | self.AVDDS | self.CS)
        time.sleep(0.010)

        # CTRL2: output data rate
        rate_bits = self.RATE_ENC.get(NAU7802_SAMPLE_RATE, 3) * 4
        b.write_reg(self.REG_CTRL2, rate_bits)

        # Internal zero-scale calibration (CTRL2[1:0] = 10)
        b.write_reg(self.REG_CTRL2, rate_bits | 0x02)
        # Poll for calibration complete (CALS bit [2] clears when done)
        deadline = time.time() + 1.0
        while time.time() < deadline:
            time.sleep(0.010)
            ctrl2 = b.read_reg(self.REG_CTRL2)
            if not (ctrl2 & 0x04):
                break
        else:
            print(f"NAU7802 {self._label}: WARNING — calibration timed out")

        print(f"NAU7802 {self._label}: OK  "
              f"gain={NAU7802_GAIN}  rate={NAU7802_SAMPLE_RATE} SPS")
        return True

    def read(self) -> int:
        """
        Poll conversion ready then burst-read the 24-bit ADC result.
        Times out after 20 ms, returns 0 on timeout.
        Returns signed 24-bit integer counts.
        """
        b = self._bus
        # Poll CR bit — timeout after 20 ms (one period at 80 SPS ≈ 12.5 ms)
        deadline = time.time() + 0.020
        while time.time() < deadline:
            pu = b.read_reg(self.REG_PU_CTRL)
            if pu & self.CR:
                break
            time.sleep(0.001)
        else:
            return 0

        raw = b.read_burst(self.REG_ADCO_B2, 3)
        return LJMi2c.to_int24(raw[0], raw[1], raw[2])


# ==============================================================================
# ICM-20948 DRIVER
# ==============================================================================

class ICM20948:
    """
    Driver for the ICM-20948 9-axis IMU.
    Handles bank-switched register access, init, and 6-axis burst reads.
    """

    # Bank-select register (accessible from all banks)
    REG_BANK_SEL = 0x7F

    # Bank 0 registers
    REG_WHO_AM_I    = 0x00   # Expected: 0xEA
    REG_PWR_MGMT_1  = 0x06
    REG_PWR_MGMT_2  = 0x07
    REG_ACCEL_XOUT_H= 0x2D   # Burst read ×6: Accel XYZ
    REG_GYRO_XOUT_H = 0x33   # Burst read ×6: Gyro XYZ

    # Bank 2 registers
    REG_ACCEL_CONFIG = 0x14
    REG_GYRO_CONFIG  = 0x01

    ACCEL_FS = {2: 0x00, 4: 0x02, 8: 0x04, 16: 0x06}
    GYRO_FS  = {250: 0x00, 500: 0x02, 1000: 0x04, 2000: 0x06}

    def __init__(self, bus: LJMi2c):
        self._bus = bus

    def _select_bank(self, bank: int):
        # Bank number encoded in bits [5:4] of BANK_SEL
        self._bus.write_reg(self.REG_BANK_SEL, bank * 16)

    def init(self) -> bool:
        """Soft reset, wake, identity check, configure accel/gyro ranges."""
        b = self._bus

        self._select_bank(0)
        b.write_reg(self.REG_PWR_MGMT_1, 0x80)   # Soft reset
        time.sleep(0.100)
        b.write_reg(self.REG_PWR_MGMT_1, 0x01)   # Wake, auto-select clock
        time.sleep(0.010)

        self._select_bank(0)
        who = b.read_reg(self.REG_WHO_AM_I)
        if who != 0xEA:
            print(f"ICM-20948: ERROR — WHO_AM_I=0x{who:02X} expected 0xEA. "
                  f"Check wiring and AD0 pin (HIGH=0x69, LOW=0x68).")
            return False

        b.write_reg(self.REG_PWR_MGMT_2, 0x00)   # Enable all axes

        self._select_bank(2)
        b.write_reg(self.REG_ACCEL_CONFIG,
                    self.ACCEL_FS.get(ICM_ACCEL_RANGE_G, 0x02))
        b.write_reg(self.REG_GYRO_CONFIG,
                    self.GYRO_FS.get(ICM_GYRO_RANGE_DPS, 0x02))

        self._select_bank(0)
        print(f"ICM-20948: OK  "
              f"accel=±{ICM_ACCEL_RANGE_G}g  "
              f"gyro=±{ICM_GYRO_RANGE_DPS} dps  "
              f"addr=0x{ICM_ADDR:02X}")
        return True

    def read(self) -> Tuple[int, int, int, int, int, int]:
        """
        Burst read accel XYZ (6 bytes) and gyro XYZ (6 bytes).
        Returns (ax, ay, az, gx, gy, gz) as signed 16-bit raw counts.
        Apply ICM_ACCEL_SCALE / ICM_GYRO_SCALE on host for physical units.
        """
        b = self._bus
        self._select_bank(0)
        a = b.read_burst(self.REG_ACCEL_XOUT_H, 6)
        g = b.read_burst(self.REG_GYRO_XOUT_H,  6)
        return (
            LJMi2c.to_int16(a[0], a[1]),
            LJMi2c.to_int16(a[2], a[3]),
            LJMi2c.to_int16(a[4], a[5]),
            LJMi2c.to_int16(g[0], g[1]),
            LJMi2c.to_int16(g[2], g[3]),
            LJMi2c.to_int16(g[4], g[5]),
        )


# ==============================================================================
# SENSOR READER BASE CLASS
# ==============================================================================

class SensorReader(ABC):
    """
    Base class for mode-specific sensor readers.
    Subclasses implement init_hardware() and read_frame().
    """

    def __init__(self, handle, ljm, active_channels: List[ChannelConfig]):
        self._handle   = handle
        self._ljm      = ljm
        self._channels = [ch for ch in active_channels if not ch.is_timestamp]
        self._tags     = {ch.key: SensorTag(ch.register) for ch in self._channels}

    @abstractmethod
    def init_hardware(self):
        """Initialize all hardware for this mode. Called once at startup."""
        pass

    @abstractmethod
    def read_frame(self) -> Dict[str, float]:
        """
        Read all active channels. Returns {channel_key: scaled_physical_value}.
        Called once per loop iteration.
        """
        pass

    def _configure_ain(self, channels: range = range(4)):
        """Configure AIN range and resolution for specified channels."""
        names  = []
        values = []
        for ch in channels:
            names.append(f"AIN{ch}_RANGE")
            values.append(AIN_RANGE_VOLTS)
            names.append(f"AIN{ch}_RESOLUTION_INDEX")
            values.append(float(AIN_RESOLUTION_INDEX))
        self._ljm.eWriteNames(self._handle, len(names), names, values)

    def _configure_rpm_ef(self):
        """
        Configure DIO-EF Index 3 (Frequency In) on RPM_PIN.

        REGISTER MAPPING — confirmed against LabJack T7 datasheet §13.2.5:
          DIO#_EF_ENABLE   : 0 = disable, 1 = enable  (NOT the mode/index number)
          DIO#_EF_INDEX    : feature selector — 3 = rising-edge period, 4 = falling
          DIO#_EF_CONFIG_A : bit1: 0 = one-shot, 1 = continuous (other bits reserved)
          DIO#_EF_CLOCK_SOURCE : 0 = Clock0 (80 MHz, 32-bit) — default, leave alone

        CORRECT CONFIGURATION SEQUENCE (mandatory order):
          1. ENABLE = 0       Disable — required before INDEX can be changed
          2. INDEX  = 3       Select Frequency In, rising edges
          3. CONFIG_A = 2     Continuous mode (bit1 = 1) — keeps updating every edge
          4. ENABLE = 1       Enable — start counting

        PREVIOUS BUG: Earlier code wrote 3 to ENABLE instead of INDEX.
        ENABLE only accepts 0/1; writing 3 is treated as 1 (enable) without
        setting any mode, leaving the EF at whatever index was default (0 = PWM
        output), which silently produces no frequency measurement.

        READ REGISTER — DIO#_EF_READ_A returns the period in CLOCK TICKS at
        80 MHz.  Conversion: freq_hz = 80_000_000 / ticks.  See _read_rpm().
        DIO#_EF_READ_B_F returns frequency in Hz directly but requires
        READ_A to be read first to latch READ_B (capture register).

        ACTIVE-LOW SENSOR NOTE:
        Index 3 triggers on rising edges.  With the active-low Hall sensor
        (normally HIGH, pulses LOW), rising edges occur at the trailing edge
        of each low pulse — one per revolution.  Index 3 is correct; no
        inversion needed.
        """
        pin = RPM_PIN
        ljm = self._ljm
        h   = self._handle
        # Step 1: disable before changing index (T7 will error if enabled)
        ljm.eWriteName(h, f"DIO{pin}_EF_ENABLE",   0)
        # Step 2: set feature index — 3 = Frequency In (rising edges)
        ljm.eWriteName(h, f"DIO{pin}_EF_INDEX",    3)
        # Step 3: continuous mode so it keeps updating on every edge
        ljm.eWriteName(h, f"DIO{pin}_EF_CONFIG_A", 2)
        # Step 4: enable
        ljm.eWriteName(h, f"DIO{pin}_EF_ENABLE",   1)

    def _configure_encoder_ef(self):
        """
        Configure DIO-EF Index 10 (Quadrature Input) on ENC_CH_A_PIN / B_PIN.

        Per LabJack T7 datasheet §13.2.10:
          DIO#_EF_INDEX  = 10  on the A channel pin only
          DIO#_EF_ENABLE = 1   on the A channel pin to start
          The B channel pin is automatically claimed by the hardware when
          Index 10 is enabled on the adjacent even-numbered A pin.
          Both pins must be disabled before configuring.
        """
        a, b = ENC_CH_A_PIN, ENC_CH_B_PIN
        ljm = self._ljm
        h   = self._handle
        # Disable both channels first
        ljm.eWriteName(h, f"DIO{a}_EF_ENABLE", 0)
        ljm.eWriteName(h, f"DIO{b}_EF_ENABLE", 0)
        # Set index on A channel, zero the count offset
        ljm.eWriteName(h, f"DIO{a}_EF_INDEX",    10)
        ljm.eWriteName(h, f"DIO{a}_EF_CONFIG_A",  0)   # count offset = 0
        # Enable A channel — B is automatically paired
        ljm.eWriteName(h, f"DIO{a}_EF_ENABLE",    1)

    def _read_rpm(self) -> float:
        """
        Read DIO-EF Index 3 period and convert to RPM.

        REGISTER USED — DIO#_EF_READ_A_AND_RESET:
          Identical to READ_A (returns period in 80 MHz clock ticks) but
          atomically clears the register to 0 after the read.  The next
          read returns 0 until a complete new edge pair has been measured.

          This is the correct register for RPM display — it ensures that
          when the shaft stops (no new edges arrive), the next poll returns
          0 and RPM drops to 0.0 immediately rather than holding the last
          valid reading indefinitely.

          READ_A (plain) holds the last period forever and was the cause of
          the "RPM stays at last value after shaft stops" bug.

        CONVERSION:
            ticks   = READ_A_AND_RESET           (e.g. 9,638,554 at ~498 RPM)
            freq_hz = 80_000_000 / ticks
            rpm     = (freq_hz / RPM_PULSES_PER_REV) * 60

        Returns 0.0 if ticks == 0 (register was cleared — no new edge pair
        since last read, i.e. shaft has stopped or not yet started).
        """
        ticks = self._ljm.eReadName(
            self._handle, f"DIO{RPM_PIN}_EF_READ_A_AND_RESET"
        )
        if ticks != ticks or ticks <= 0:
            return 0.0
        freq_hz = 80_000_000.0 / ticks
        return (freq_hz / RPM_PULSES_PER_REV) * 60.0

    def _read_encoder_counts(self) -> int:
        """Read DIO-EF quadrature count from Ch A register."""
        return int(self._ljm.eReadName(
            self._handle, f"DIO{ENC_CH_A_PIN}_EF_READ_A"
        ))


# ==============================================================================
# MODE-SPECIFIC SENSOR READERS
# ==============================================================================

class SuspensionReader(SensorReader):
    """
    Mode 1: Linear pots (AIN0–3), load cells (NAU7802 ×4), IMU (ICM-20948).
    """

    def __init__(self, handle, ljm, active_channels):
        super().__init__(handle, ljm, active_channels)
        self._nau: Dict[int, NAU7802] = {}
        self._icm: Optional[ICM20948] = None
        self._imu_ok = False

    def init_hardware(self):
        # AIN0–3 range configuration
        self._configure_ain(range(4))

        # NAU7802 — only init buses needed by active channels
        nau_tags = {
            SensorTag.NAU7802_BUS0.value: 0,
            SensorTag.NAU7802_BUS1.value: 1,
            SensorTag.NAU7802_BUS2.value: 2,
            SensorTag.NAU7802_BUS3.value: 3,
        }
        needed_buses = set()
        for ch in self._channels:
            if ch.register in nau_tags:
                needed_buses.add(nau_tags[ch.register])

        for bus_idx in sorted(needed_buses):
            sda, scl = NAU7802_BUSES[bus_idx]
            bus = LJMi2c(self._handle, self._ljm, sda, scl, NAU7802_ADDR)
            label = ["FL", "FR", "BL", "BR"][bus_idx]
            nau = NAU7802(bus, f"LC{bus_idx}-{label}")
            nau.init()
            self._nau[bus_idx] = nau

        # ICM-20948 — only if any IMU channel is active
        icm_tags = {SensorTag.ICM_ACCEL_X.value, SensorTag.ICM_ACCEL_Y.value,
                    SensorTag.ICM_ACCEL_Z.value, SensorTag.ICM_GYRO_X.value,
                    SensorTag.ICM_GYRO_Y.value,  SensorTag.ICM_GYRO_Z.value}
        if any(ch.register in icm_tags for ch in self._channels):
            bus = LJMi2c(self._handle, self._ljm, ICM_SDA, ICM_SCL, ICM_ADDR)
            self._icm = ICM20948(bus)
            self._imu_ok = self._icm.init()

    def read_frame(self) -> Dict[str, float]:
        values: Dict[str, float] = {}

        # --- AIN batch read (all 4 in one LJM call) --------------------------
        ain_needed = [ch for ch in self._channels
                      if SensorTag(ch.register) in
                      (SensorTag.AIN0, SensorTag.AIN1,
                       SensorTag.AIN2, SensorTag.AIN3)]
        if ain_needed:
            ain_names = [f"AIN{SensorTag(ch.register).value}" for ch in ain_needed]
            ain_vals  = self._ljm.eReadNames(
                self._handle, len(ain_names), ain_names
            )
            for ch, raw in zip(ain_needed, ain_vals):
                values[ch.key] = raw * ch.scale + ch.offset

        # --- NAU7802 reads (sequential per bus) ------------------------------
        nau_map = {
            SensorTag.NAU7802_BUS0.value: 0,
            SensorTag.NAU7802_BUS1.value: 1,
            SensorTag.NAU7802_BUS2.value: 2,
            SensorTag.NAU7802_BUS3.value: 3,
        }
        for ch in self._channels:
            if ch.register in nau_map:
                bus_idx = nau_map[ch.register]
                if bus_idx in self._nau:
                    raw = self._nau[bus_idx].read()
                    values[ch.key] = raw * ch.scale + ch.offset

        # --- ICM-20948 burst read (all 6 axes in one transaction) ------------
        icm_map = {
            SensorTag.ICM_ACCEL_X.value: 0,
            SensorTag.ICM_ACCEL_Y.value: 1,
            SensorTag.ICM_ACCEL_Z.value: 2,
            SensorTag.ICM_GYRO_X.value:  3,
            SensorTag.ICM_GYRO_Y.value:  4,
            SensorTag.ICM_GYRO_Z.value:  5,
        }
        icm_needed = [ch for ch in self._channels if ch.register in icm_map]
        if icm_needed and self._imu_ok and self._icm:
            ax, ay, az, gx, gy, gz = self._icm.read()
            imu_raw = [ax, ay, az, gx, gy, gz]
            for ch in icm_needed:
                raw = imu_raw[icm_map[ch.register]]
                values[ch.key] = raw * ch.scale + ch.offset

        return values


class DrivetrainReader(SensorReader):
    """
    Mode 2: Load cell (NAU7802 bus 0), Hall effect RPM (FIO0).
    Writes RPM to USER_RAM0_F32 after each read so the T7 Lua stepper
    control script can consume it independently.

    RPM SAMPLE-AND-HOLD:
    READ_A_AND_RESET returns 0 whenever no new edge pair has completed
    since the last read — i.e. every poll that lands between two rising
    edges while the shaft is turning gives 0.  At low RPM (shaft period
    longer than the acquisition interval) this produces spiky single-sample
    bursts rather than a smooth reading.

    The sample-and-hold logic here:
      - Holds the last non-zero RPM reading
      - Starts a 5-second linear decay to zero once no new reading arrives
      - The decay slope is last_rpm / RPM_HOLDOFF_S per second
      - Once the holdoff expires the output is clamped to 0.0

    This gives visually smooth behaviour: the reading stays stable while
    the shaft is turning faster than the acquisition rate, and ramps
    smoothly to zero over 5 s after the shaft stops.
    """

    RPM_HOLDOFF_S = 5.0   # seconds before RPM decays to zero after last reading

    def __init__(self, handle, ljm, active_channels):
        super().__init__(handle, ljm, active_channels)
        self._nau: Optional[NAU7802] = None
        self._last_rpm:      float = 0.0   # last valid non-zero RPM
        self._last_rpm_time: float = 0.0   # perf_counter timestamp of last valid read
        self._held_rpm:      float = 0.0   # current output after hold/decay

    def init_hardware(self):
        # NAU7802 bus 0 only
        sda, scl = NAU7802_BUSES[0]
        bus = LJMi2c(self._handle, self._ljm, sda, scl, NAU7802_ADDR)
        self._nau = NAU7802(bus, "LC0-FL")
        self._nau.init()

        # Hall effect RPM DIO-EF Mode 3
        self._configure_rpm_ef()
        print(f"Hall Effect RPM: OK  FIO{RPM_PIN}  "
              f"window={RPM_WINDOW_US} µs  "
              f"PPR={RPM_PULSES_PER_REV}")
        print("  !! Hardware: 3.3 kΩ pull-up to 3.3V required on FIO0 !!")

    def read_frame(self) -> Dict[str, float]:
        values: Dict[str, float] = {}

        for ch in self._channels:
            tag = SensorTag(ch.register)

            if tag == SensorTag.NAU7802_BUS0 and self._nau:
                raw = self._nau.read()
                values[ch.key] = raw * ch.scale + ch.offset

            elif tag == SensorTag.HALL_RPM:
                rpm = self._read_rpm_held()
                values[ch.key] = rpm * ch.scale + ch.offset

                # Write to USER_RAM0_F32 for T7 Lua stepper control loop
                try:
                    self._ljm.eWriteName(
                        self._handle, STEPPER_RPM_RAM_NAME, rpm
                    )
                except Exception:
                    pass

        return values

    def _read_rpm_held(self) -> float:
        """
        Read RPM with sample-and-hold and 5-second linear decay.

        If READ_A_AND_RESET returns a valid non-zero period, latch it and
        update the timestamp.  If it returns 0 (no new edge pair completed
        since last read), compute the output from the held value minus
        linear decay progress.  Clamps to 0.0 when holdoff expires.
        """
        raw_rpm = self._read_rpm()
        now = time.perf_counter()

        if raw_rpm > 0.0:
            # Fresh reading — latch and reset holdoff clock
            self._last_rpm      = raw_rpm
            self._last_rpm_time = now
            self._held_rpm      = raw_rpm
        else:
            # No new edge — apply linear decay from last valid reading
            elapsed = now - self._last_rpm_time
            if elapsed >= self.RPM_HOLDOFF_S or self._last_rpm == 0.0:
                self._held_rpm = 0.0
            else:
                decay = self._last_rpm * (elapsed / self.RPM_HOLDOFF_S)
                self._held_rpm = max(0.0, self._last_rpm - decay)

        return self._held_rpm


class SteeringReader(SensorReader):
    """
    Mode 3: Load cell (NAU7802 bus 0), rotary encoder (FIO2/3).
    Velocity is derived as first-difference of encoder angle per second.
    """

    def __init__(self, handle, ljm, active_channels):
        super().__init__(handle, ljm, active_channels)
        self._nau: Optional[NAU7802] = None
        self._prev_counts: int   = 0
        self._prev_time: float   = 0.0

    def init_hardware(self):
        # NAU7802 bus 0
        sda, scl = NAU7802_BUSES[0]
        bus = LJMi2c(self._handle, self._ljm, sda, scl, NAU7802_ADDR)
        self._nau = NAU7802(bus, "LC0-FL")
        self._nau.init()

        # Rotary encoder DIO-EF Mode 10
        self._configure_encoder_ef()
        print(f"Rotary Encoder: OK  FIO{ENC_CH_A_PIN}/FIO{ENC_CH_B_PIN}  "
              f"PPR={round(ENC_COUNTS_PER_DEG * 360 / 4)}  "
              f"counts/deg={ENC_COUNTS_PER_DEG:.4f}")
        print("  !! Set ENC_PPR in config.py once encoder model is confirmed !!")

        self._prev_counts = self._read_encoder_counts()
        self._prev_time   = time.perf_counter()

    def read_frame(self) -> Dict[str, float]:
        values: Dict[str, float] = {}
        counts = self._read_encoder_counts()
        now    = time.perf_counter()

        for ch in self._channels:
            tag = SensorTag(ch.register)

            if tag == SensorTag.NAU7802_BUS0 and self._nau:
                raw = self._nau.read()
                values[ch.key] = raw * ch.scale + ch.offset

            elif tag == SensorTag.ENC_COUNTS:
                values[ch.key] = float(counts)

            elif tag == SensorTag.ENC_ANGLE:
                angle = counts / ENC_COUNTS_PER_DEG
                values[ch.key] = angle * ch.scale + ch.offset

            elif tag == SensorTag.ENC_VELOCITY:
                dt = now - self._prev_time
                if dt > 0:
                    delta_counts = counts - self._prev_counts
                    deg_per_s    = (delta_counts / ENC_COUNTS_PER_DEG) / dt
                    values[ch.key] = deg_per_s * ch.scale + ch.offset
                else:
                    values[ch.key] = 0.0

        self._prev_counts = counts
        self._prev_time   = now
        return values


# ==============================================================================
# ACQUISITION THREAD
# Interface to dashboard.py is identical to v1 — same constructor args,
# same start/stop, same DataFrame output into RingBuffer.
# ==============================================================================

class AcquisitionThread(threading.Thread):
    """
    Background daemon thread. Connects to T7, initializes hardware for the
    active test mode, polls sensors at the configured rate, and pushes
    DataFrames into the ring buffer for the UI to consume.

    Filter parameters control the acquisition-layer ChannelFilter applied to
    every data channel before values enter the ring buffer.  These affect both
    the live display AND the CSV log.  For display-only smoothing, configure
    the per-card filter in ChannelValueCard independently.
    """

    def __init__(
        self,
        mode: TestMode,
        active_channels: List[ChannelConfig],
        sample_rate_hz: float,
        buffer: RingBuffer,
        status_callback: Callable[[AcqStatus, str], None],
        ip_address: str = T7_IP_ADDRESS,
        connection_type: str = T7_CONNECTION,
        filter_profile:    FilterProfile = FilterProfile.NONE,
        filter_n:          int           = 5,
        filter_ema_alpha:  float         = FILTER_EMA_ALPHA,
        filter_ema2_alpha: float         = FILTER_EMA2_ALPHA,
    ):
        super().__init__(daemon=True, name="AcquisitionThread")
        self.mode              = mode
        self.active_channels   = active_channels
        self.sample_rate_hz    = sample_rate_hz
        self.buffer            = buffer
        self.status_callback   = status_callback
        self.ip_address        = ip_address
        self.connection_type   = connection_type
        self.filter_profile    = filter_profile
        self.filter_n          = filter_n
        self.filter_ema_alpha  = filter_ema_alpha
        self.filter_ema2_alpha = filter_ema2_alpha

        self._stop_event = threading.Event()
        self._handle     = None
        self._ljm        = None

    def stop(self):
        self._stop_event.set()

    def run(self):
        try:
            self._connect()
            self._acquire_loop()
        except Exception as e:
            self.status_callback(AcqStatus.ERROR, f"Acquisition error: {e}")
        finally:
            self._disconnect()

    # --------------------------------------------------------------------------
    def _connect(self):
        self.status_callback(
            AcqStatus.CONNECTING,
            f"Connecting to T7 at {self.ip_address}..."
        )
        try:
            from labjack import ljm
            self._ljm = ljm

            # openS() connects directly by IP string, bypassing LJM UDP
            # broadcast discovery. Required when VPN software (e.g. Tailscale)
            # is active — VPNs intercept UDP broadcasts before they reach the
            # physical ethernet adapter, causing LJME_DEVICE_NOT_FOUND even
            # when the T7 is reachable by ping. openS() avoids this entirely.
            if self.connection_type.upper() == "USB":
                self._handle = ljm.openS("T7", "USB", "ANY")
            else:
                self._handle = ljm.openS("T7", "ETHERNET", self.ip_address)

            fw = ljm.eReadName(self._handle, "FIRMWARE_VERSION")
            self.status_callback(
                AcqStatus.CONNECTED,
                f"Connected — T7 firmware {fw:.4f}"
            )

        except ImportError:
            raise RuntimeError(
                "labjack-ljm not found. Run: pip install labjack-ljm\n"
                "Also install the LJM C driver from labjack.com."
            )
        except Exception as e:
            raise RuntimeError(f"Failed to connect to T7: {e}")

    # --------------------------------------------------------------------------
    def _acquire_loop(self):
        ljm    = self._ljm
        handle = self._handle

        # Instantiate the correct reader for this mode
        reader_map = {
            TestMode.SUSPENSION: SuspensionReader,
            TestMode.DRIVETRAIN: DrivetrainReader,
            TestMode.STEERING:   SteeringReader,
        }
        ReaderClass = reader_map[self.mode]
        reader = ReaderClass(handle, ljm, self.active_channels)

        # Hardware init
        self.status_callback(AcqStatus.CONNECTING, "Initializing hardware...")
        try:
            reader.init_hardware()
        except Exception as e:
            raise RuntimeError(f"Hardware init failed: {e}")

        # Warm-up guard
        self.status_callback(
            AcqStatus.CONNECTING,
            f"Warming up sensors ({WARMUP_SECONDS} s)..."
        )
        warmup_end = time.time() + WARMUP_SECONDS
        while time.time() < warmup_end and not self._stop_event.is_set():
            time.sleep(0.1)

        if self._stop_event.is_set():
            return

        # Build one ChannelFilter per non-timestamp channel.
        # Profile and parameters come from the channel's filter_profile /
        # filter_n fields if present (future config.py extension), otherwise
        # default to the thread-level filter_profile and filter_n args.
        data_channels = [ch for ch in self.active_channels if not ch.is_timestamp]
        acq_filters: Dict[str, ChannelFilter] = {
            ch.key: ChannelFilter(
                profile    = self.filter_profile,
                sma_n      = self.filter_n,
                ema_alpha  = self.filter_ema_alpha,
                ema2_alpha = self.filter_ema2_alpha,
            )
            for ch in data_channels
        }

        interval_s = 1.0 / self.sample_rate_hz
        t_start    = time.perf_counter()

        self.status_callback(
            AcqStatus.STREAMING,
            f"Streaming {len(data_channels)} channels @ {self.sample_rate_hz} Hz"
            f"  |  Acq filter: {self.filter_profile.value}"
            + (f" N={self.filter_n}" if self.filter_profile != FilterProfile.NONE else "")
        )

        while not self._stop_event.is_set():
            loop_start = time.perf_counter()

            try:
                raw_values = reader.read_frame()
                t7_ts_ms   = (loop_start - t_start) * 1000.0

                # Apply acquisition-layer filter to each channel value.
                # Filtered values go into the DataFrame → ring buffer → CSV.
                filtered_values = {
                    key: acq_filters[key].push(val) if key in acq_filters else val
                    for key, val in raw_values.items()
                }

                frame = DataFrame(
                    wall_time       = time.time(),
                    t7_timestamp_ms = t7_ts_ms,
                    values          = filtered_values,
                )
                self.buffer.push(frame)

            except Exception as e:
                self.status_callback(AcqStatus.ERROR,
                                     f"Read error (retrying): {e}")
                time.sleep(0.5)
                continue

            # Precise sleep to maintain sample rate
            elapsed   = time.perf_counter() - loop_start
            sleep_for = interval_s - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    # --------------------------------------------------------------------------
    def _disconnect(self):
        if self._handle is not None and self._ljm is not None:
            try:
                self._ljm.close(self._handle)
            except Exception:
                pass
            self._handle = None
        self.status_callback(AcqStatus.DISCONNECTED, "Disconnected")


# ==============================================================================
# CSV LOGGER (unchanged from v1)
# ==============================================================================

class CSVLogger:
    """
    Writes DataFrames to a CSV file. Thread-safe for single UI thread writer.
    """

    def __init__(self):
        self._file        = None
        self._writer      = None
        self._lock        = threading.Lock()
        self._path: Optional[str] = None
        self._frame_count = 0
        self._channels: List[ChannelConfig] = []

    @property
    def is_open(self) -> bool:
        return self._file is not None

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def path(self) -> Optional[str]:
        return self._path

    def open(
        self,
        path: str,
        channels: List[ChannelConfig],
        mode: TestMode,
        session_meta: Dict[str, str],
    ):
        import csv
        with self._lock:
            if self._file:
                self.close()

            self._path     = path
            self._channels = [ch for ch in channels if not ch.is_timestamp]
            self._file     = open(path, "w", newline="", encoding="utf-8")
            self._writer   = csv.writer(self._file)
            self._frame_count = 0

            # Metadata comment header
            self._file.write(f"# Vehicle Sensor Suite — Data Log\n")
            self._file.write(f"# Mode: {mode.value}\n")
            self._file.write(f"# Date: {datetime.datetime.now().isoformat()}\n")
            for k, v in session_meta.items():
                self._file.write(f"# {k}: {v}\n")
            self._file.write("#\n# Scale factors applied: raw * scale + offset\n")
            for ch in self._channels:
                self._file.write(
                    f"#   {ch.key}: scale={ch.scale}, "
                    f"offset={ch.offset}, unit={ch.unit}\n"
                )
            self._file.write("#\n")

            headers = (
                ["wall_time_iso", "elapsed_ms"] +
                [f"{ch.key}_{ch.unit}" for ch in self._channels]
            )
            self._writer.writerow(headers)
            self._file.flush()

    def write_frame(self, frame: DataFrame):
        if not self.is_open:
            return
        with self._lock:
            try:
                iso = datetime.datetime.fromtimestamp(
                    frame.wall_time
                ).strftime("%Y-%m-%d %H:%M:%S.%f")
                row = (
                    [iso, f"{frame.t7_timestamp_ms:.3f}"] +
                    [f"{frame.values.get(ch.key, 0.0):.6f}"
                     for ch in self._channels]
                )
                self._writer.writerow(row)
                self._frame_count += 1
                if self._frame_count % 50 == 0:
                    self._file.flush()
            except Exception:
                pass

    def close(self):
        with self._lock:
            if self._file:
                try:
                    self._file.flush()
                    self._file.close()
                except Exception:
                    pass
                self._file   = None
                self._writer = None
