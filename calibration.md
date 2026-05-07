# Baja SAE Test Platform — Sensor Calibration Guide

<div align="center">

![Version](https://img.shields.io/badge/Dashboard-v2.6.3-blue?style=for-the-badge)
![Standard](https://img.shields.io/badge/Standard-ASTM%20E74-darkgreen?style=for-the-badge)
![Hardware](https://img.shields.io/badge/Hardware-NAU7802%2024--bit-E74C3C?style=for-the-badge)

**Engineer Control Panel calibration reference for load cells, potentiometers, and derived channels**  
*Accessed via the Engineer button in the dashboard sidebar*

</div>

---

## Overview

Calibration in the Baja VTD dashboard is a per-channel linear mapping from raw ADC counts to physical engineering units. The mapping takes the form:

```
Physical Value = (Raw ADC Count × Scale) + Offset
```

Calibration constants are entered through the **Engineer Control Panel**, persisted to `svss_engineer.json`, and applied automatically on every startup and mode switch. Source files are never modified. The default state for all channels is `Scale = 1.0`, `Offset = 0.0`, which passes raw counts to the display unchanged.

> **Note:** Calibration changes take effect on the display immediately when **Apply** is pressed. The acquisition layer (CSV output) picks up the new constants after the stream is restarted. Any data recorded before a stream restart reflects the prior calibration constants.

---

## Accessing the Engineer Control Panel

1. Launch the dashboard and select the appropriate test mode.
2. Click **Engineer** in the control sidebar.
3. Enter the engineer password at the prompt.
4. Locate the channel to be calibrated in the scrollable table.
5. Enter the computed **Scale** and **Offset** values and press **Apply**.
6. Stop and restart the stream to commit the new constants to the acquisition layer.

> **Note:** The default password is set on first launch. It can be changed from within the panel using the **Change Password** button. The password hash is stored in `svss_engineer.json`. To reset a lost password, delete `svss_engineer.json` — this also clears all saved calibration constants, so record values before deleting.

---

## Load Cell Calibration (NAU7802)

Load cell calibration is performed using a **two-point linear method** in accordance with **ASTM E74**, *Standard Practice for Calibration of Force-Measuring Instruments for Verifying the Force Indication of Testing Machines*.

### Required Equipment

| Item | Requirement |
|---|---|
| Reference weights or force standard | NIST-traceable, covering the expected operating load range |
| Stable, level surface | For vertical application of reference load |
| Multimeter or oscilloscope | Optional — for verifying sensor signal presence |

### Procedure

> **Important:** Perform calibration with the load cell installed in its final mechanical configuration. All wiring, mounting hardware, and mechanical preloads must match the intended test condition.

**Step 1 — Establish a raw zero-load baseline**

1. Ensure all hardware is powered and connected. Launch the dashboard and select the test mode containing the load cell channel to be calibrated.
2. Press **Start Stream** and allow the 6-second hardware warmup to complete.
3. In the sidebar, set the **Acq Filter (CSV)** profile to **None** to observe unfiltered raw counts.
4. With no load applied to the load cell, open the Engineer Control Panel.
5. Record the current raw count reading for the channel. This is **C_zero**.

**Step 2 — Apply a known reference load**

1. Apply a known reference load **F_ref** (in the desired output unit — lbf or N) to the load cell along its sensitive axis.
2. Allow the reading to stabilize for a minimum of **5 seconds**.
3. Record the raw count reading. This is **C_ref**.

**Step 3 — Compute calibration constants**

```
Scale  =  F_ref  /  (C_ref − C_zero)

Offset =  −(C_zero × Scale)
```

**Step 4 — Enter and verify**

1. Enter the computed **Scale** and **Offset** into the Engineer Control Panel for the channel. Press **Apply**.
2. Confirm the display reads approximately **0** under no load and **F_ref** under the reference load.
3. Acceptable tolerance: **±1%** of the reference load value.
4. Stop and restart the stream to apply the new constants to the acquisition layer.
5. Document the reference load used, raw counts recorded, computed constants, and verification result. Retain this record with the test data package.

### Multi-Point Calibration

The two-point procedure above assumes a linear response across the full operating range, which is valid for most strain gauge load cells within their rated capacity. If nonlinearity is suspected or the operating range spans a large fraction of the load cell's rated capacity, a three-point or five-point calibration is recommended:

1. Repeat the loaded measurement at **three to five evenly spaced load levels** between zero and the maximum expected operating load.
2. Fit a linear regression (least squares) to the collected (C, F) pairs.
3. The slope of the regression is **Scale**; the intercept divided by Scale yields **Offset**.

> **Note:** `NAU7802_GAIN` is fixed at 128 throughout calibration and operation. Do not modify this value — changing the PGA gain invalidates all existing calibration constants and requires full recalibration.

### Calibration Standard Reference

| Standard | Title | Applicability |
|---|---|---|
| **ASTM E74** | Standard Practice for Calibration of Force-Measuring Instruments for Verifying the Force Indication of Testing Machines | Primary standard — governs the two-point and multi-point calibration procedures described above |

Reference weights or transfer standards used in this procedure must be traceable to the **National Institute of Standards and Technology (NIST)** or an equivalent national metrology body.

---

## Linear Potentiometer Calibration (Suspension Travel)

Linear potentiometer channels (Travel FL/FR/BL/RR) output a voltage read on AIN0–3. The ADC returns a raw voltage value. Calibration maps this voltage to suspension travel distance in mm (or inches).

### Procedure

**Step 1 — Establish physical reference positions**

1. Mount the potentiometer in its final installation with the suspension at full droop (minimum travel).
2. Start the dashboard stream with the filter set to **None**.
3. Record the raw voltage reading at full droop. This is **V_droop**.
4. Move the suspension to full bump (maximum travel).
5. Record the raw voltage reading at full bump. This is **V_bump**.

**Step 2 — Compute constants**

```
Scale  =  (Travel_bump − Travel_droop)  /  (V_bump − V_droop)

Offset =  Travel_droop − (V_droop × Scale)
```

Where `Travel_bump` and `Travel_droop` are the physical travel distances in mm measured with a calibrated linear gauge or calipers.

**Step 3 — Enter and verify**

1. Enter **Scale** and **Offset** into the Engineer Control Panel for the channel. Press **Apply**.
2. Move the suspension through its range and confirm the displayed travel value tracks the physical position within ±0.5 mm.

> **Note:** Potentiometer output is only linear within the manufacturer's specified travel range. Do not apply suspension loads that drive the potentiometer beyond its mechanical stops.

---

## IMU Channel Scaling (ICM-20948)

The ICM-20948 accelerometer and gyroscope channels are scaled by fixed hardware sensitivity constants derived from the sensor datasheet. These are set as default Scale values in `config.py` and should not require field adjustment unless the IMU full-scale range register is changed.

| Channel | Default Scale | Basis |
|---|---|---|
| Accel X / Y / Z | `0.0001220703` | 16-bit ADC, ±4g range → 8192 LSB/g |
| Gyro X / Y / Z | Datasheet value | 16-bit ADC, configured full-scale range |

If the IMU full-scale range is changed in `acquisition.py`, the corresponding scale constants in `config.py` must be updated to match the new sensitivity figure from the ICM-20948 datasheet.

---

## Steering Torque Sensor

The Steering Torque channel uses NAU7802 Bus 0 (EIO0/1), shared with the Suspension Front Left load cell. The calibration procedure is identical to the load cell procedure above, substituting torque units (lbf·in or N·m) for force units.

> **Note:** When switching between Suspension and Steering test modes, the Bus 0 calibration constants in the Engineer Control Panel apply to whichever channel is active in that mode. Verify that the correct constants are loaded for the intended mode before recording data.

---

## Calibration File Reference

All calibration constants are stored in `svss_engineer.json` in the application directory. The file is created automatically on first use and is not part of the repository (listed in `.gitignore`).

```json
{
  "password_hash": "<sha256_hash>",
  "calibration": {
    "loadcell_fl": { "scale": 0.0000151514, "offset": 0.0641068900 },
    "linpot_fl":   { "scale": 15.4500000000, "offset": 0.0150000000 },
    "...": "..."
  }
}
```

> **Warning:** Do not manually edit `svss_engineer.json` while the dashboard is running. The file is read at startup and on every mode switch. Manual edits made during a session will not take effect until the dashboard is restarted, and concurrent writes may corrupt the file.

### Backup and Transfer

To transfer calibration constants to a new host PC:

1. Copy `svss_engineer.json` from the source machine's application directory to the same location on the destination machine.
2. Launch the dashboard — saved constants are applied automatically.

To reset all calibration to defaults:

1. Delete `svss_engineer.json`.
2. Relaunch the dashboard. All channels revert to `Scale = 1.0`, `Offset = 0.0`.

---

## Quick Reference — Common Failure Modes

| Symptom | Probable Cause | Action |
|---|---|---|
| Channel reads large constant value, unresponsive to load | Offset larger than sensor range, or I2C NACK during init | Verify wiring; reset Scale to `1.0`, Offset to `0.0`; re-run calibration |
| Channel reads non-zero at known zero load after calibration | Zero-load reading (C_zero) captured while residual load was present | Re-run calibration from Step 1 with load cell fully unloaded |
| Display value correct but CSV values are wrong | Stream not restarted after applying calibration | Stop stream, restart stream |
| Physical value drifts slowly over time | Thermal drift in load cell or NAU7802 | Re-zero with the **Zero** button for display; recalibrate if CSV accuracy is affected |
| All channels read zero or dashes after mode switch | `svss_engineer.json` contains a corrupt or mismatched key | Delete `svss_engineer.json` and re-enter calibration constants |
| Calibration constants lost after update | `svss_engineer.json` not present on new installation | Copy `svss_engineer.json` from the previous installation (see Backup and Transfer above) |

---

## See Also

- [README](./README.md) — Installation, architecture, and hardware reference
- [LabJack T7 Datasheet](https://labjack.com/pages/support?doc=/datasheets/t7-datasheet/) — T7 pin reference and register documentation
- [NAU7802 Datasheet](https://www.nuvoton.com/products/smart-home-audio/audio-converters/) — NAU7802 register map and sensitivity specifications
- [ICM-20948 Datasheet](https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/) — ICM-20948 full-scale range and sensitivity tables
- [ASTM E74](https://www.astm.org/e0074-23.html) — Standard Practice for Calibration of Force-Measuring Instruments

---

<div align="center">
<sub>Baja SAE Test Platform · Southern Adventist University School of Engineering & Physics · Built with LabJack LJM · PyQt6 · pyqtgraph</sub>
</div>