# Baja SAE Test Platform — DAQ Dashboard

<div align="center">

![Python](https://img.shields.io/badge/Python-3.13-3776AB?style=for-the-badge&logo=python&logoColor=white)
![PyQt6](https://img.shields.io/badge/PyQt6-6.x-41CD52?style=for-the-badge&logo=qt&logoColor=white)
![LabJack](https://img.shields.io/badge/LabJack-T7-E74C3C?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)
![Version](https://img.shields.io/badge/Dashboard-v2.5.1-blue?style=for-the-badge)

**Real-time multi-mode vehicle dynamics data acquisition and visualization suite**  
*Senior Design Project — Southern Adventist University School of Engineering & Physics*

</div>

---

## Overview

The Baja SAE Test Platform DAQ Dashboard is a Python-based real-time data acquisition and visualization application for stationary vehicle dynamics testing. The system interfaces with a **LabJack T7 DAQ** over Ethernet and supports three independent test modes, each with dedicated sensor channels and signal conditioning.

### Supported Test Modes

| Mode | Sensors | Purpose |
|---|---|---|
| **Suspension** | 4× Load Cells (NAU7802 24-bit ADC), 4× Linear Potentiometers | Measure suspension corner forces and wheel travel |
| **Drivetrain** | Hall Effect RPM Sensor | Measure drivetrain output shaft speed |
| **Steering** | Quadrature Encoder, Torque Sensor (NAU7802 24-bit ADC) | Measure steering angle, rate, and steering torque |

---

## Hardware Requirements

| Component | Details |
|---|---|
| **DAQ** | LabJack T7 at static IP `192.168.1.10` (Ethernet) |
| **24-bit ADCs** | 4× NAU7802 (Adafruit breakout) on independent soft I2C buses |
| **Linear Potentiometers** | 4× on AIN0–3 |
| **RPM Sensor** | Hall effect, NPN open-collector, active-low on FIO0 |
| **Encoder** | Quadrature encoder on FIO2/3 (DIO-EF Mode 10) |
| **Host PC** | Windows 10/11 with Python 3.13 |

> **Note:** The four NAU7802 I2C buses are shared across test modes. In Suspension mode all four channels measure corner load cells. In drivetrain mode one channel is assigned for measuring force exerted by the dynamometer on the magnetic brake. In Steering mode one channel is reassigned to the angular load cell (torque sensor); unused suspension channels are disabled in that mode.

---

## Software Architecture

```
sau_daq_dashboard/
├── config.py          # Hardware pin map, SensorTag enum, ChannelConfig, constants
├── acquisition.py     # I2C drivers, LJM interface, AcquisitionThread, FilterProfiles
├── dashboard.py       # PyQt6/pyqtgraph real-time UI, CSV logger
└── README.md
```

### Data Flow

```
NAU7802 / AIN read_frame()
  → raw × scale + offset          [calibration — config.py]
  → ChannelFilter.push()          [acquisition-layer filter — affects CSV]
  → RingBuffer → CSV
  → ChannelValueCard.update_value()
      → display ChannelFilter     [display-layer filter — UI only]
      → − display_offset          [Zero button]
      → plots
```

---

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/SethRobertson71/sau_daq_dashboard.git
cd sau_daq_dashboard
```

### 2. Create a Virtual Environment (Recommended)

```bash
python -m venv .venv

# Windows
.venv\Scripts\activate

# macOS / Linux
source .venv/bin/activate
```

### 3. Install Python Dependencies

```bash
pip install -r requirements.txt
```

**`requirements.txt` contents:**

```
PyQt6>=6.5.0
pyqtgraph>=0.13.3
numpy>=1.26.0
labjack-ljm
```

> **Note:** The `labjack-ljm` Python package is a thin wrapper around the LJM system library. The LJM driver must be installed on the host system **before** running `pip install`. See Step 4.

### 4. Install the LabJack LJM Driver

Download and install from the LabJack support page:  
👉 [https://labjack.com/pages/support](https://labjack.com/pages/support?doc=/software-driver/installer-downloads/ljm-software-installers-t4-t7-digit/)

Select the installer appropriate for your OS (Windows 10/11 x64 recommended).

### 5. Configure Network

The T7 is assigned a fixed IP address. Configure the host PC's Ethernet adapter with a static IP on the same subnet:

| Parameter | Value |
|---|---|
| Host IP | `192.168.1.x` (e.g. `192.168.1.100`) |
| Subnet Mask | `255.255.255.0` |
| T7 IP | `192.168.1.10` (fixed) |

Verify connectivity before launching:
```bash
ping 192.168.1.10
```

---

## Running the Dashboard

```bash
python dashboard.py
```

Press **Start** in the dashboard to initiate the T7 connection. A 6-second hardware warmup period runs automatically before data streaming begins.

---

## Pin Assignment Reference

| T7 Pin | Signal | Mode | Notes |
|---|---|---|---|
| `AIN0–3` | Linear Pots FL/FR/RL/RR | Suspension | Wheel travel — 0–10V range |
| `FIO0` | Hall Effect RPM | Drivetrain | Active-low, DIO-EF Index 3 |
| `FIO2/3` | Quadrature Encoder A/B | Steering | DIO-EF Index 10 |
| `EIO0/1` | NAU7802 Bus0 SDA/SCL | Suspension / Steering | LC0 — Front Left / Torque Sensor |
| `EIO2/3` | NAU7802 Bus1 SDA/SCL | Suspension | LC1 — Front Right |
| `EIO4/5` | NAU7802 Bus2 SDA/SCL | Suspension | LC2 — Rear Left |
| `EIO6/7` | NAU7802 Bus3 SDA/SCL | Suspension | LC3 — Rear Right |

> Bus0 (EIO0/1) serves as the suspension Front Left channel in Suspension mode and is reassigned to the steering torque sensor in Steering mode.

---

## CSV Export

Test data is saved to the directory configured in `config.py` (`CSV_DEFAULT_DIR`). Each export includes:

- Metadata header (timestamp, dashboard version, test mode, sample rate)
- Column headers matching `ChannelConfig.key` values
- Acquisition-filtered values only — display-layer Zero offsets are **not** written to file

---

## Version History

| Version | Notes |
|---|---|
| `v2.5.1` | Initial release — per-card display filter + Zero button, acquisition-layer SMA filter, CSV export with metadata headers, dark/light theme toggle, horizontally scrollable fixed-width channel cards, cross-plot widget, channel selector panel, tabbed plot layout (Time Series / Cross Plot) |

---

## Notes

- `NAU7802_GAIN = 128` is fixed throughout calibration and operation — do not modify.
- `DIO#_EF_ENABLE` is a **boolean** register (0 or 1 only). Mode selection requires writing to `DIO#_EF_INDEX`. Writing a mode index to `ENABLE` silently misbehaves.
- The T7 connection always uses a direct IP string — do not substitute `"ANY"` as the connection identifier.
- See [www.support.labjack.com](https://support.labjack.com/docs/t-series-datasheet) for further support on the LabJack T7.

---

## Contact

**Organization:** Southern Adventist University — School of Engineering & Physics  
**Point of Contact:** Seth Robertson  
**Email:** [sethnrobertson@southern.edu](mailto:sethnrobertson@southern.edu)

For questions, bug reports, or collaboration inquiries, open an issue on this repository or reach out directly.

---

## License

Copyright (c) 2025 Seth Robertson — Southern Adventist University School of Engineering & Physics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

---

<div align="center">
<sub>Built with LabJack LJM · PyQt6 · pyqtgraph</sub>
</div>
