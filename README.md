# 🌍 Earthquake Detection System v5.3

> A real-time earthquake and fire detection system built on the ESP32, using a machine learning-inspired multi-feature classification model running on-device. Continuously analyzes seismic sensor data to detect hazardous events, trigger automatic power cutoff, and dispatch SMS alerts.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Wiring / Pin Configuration](#wiring--pin-configuration)
- [Software Dependencies](#software-dependencies)
- [Configuration](#configuration)
- [ML Detection Pipeline](#ml-detection-pipeline)
- [System States](#system-states)
- [SMS Alert System](#sms-alert-system)
- [OLED Display Screens](#oled-display-screens)
- [Serial Monitor Output](#serial-monitor-output)
- [Getting Started](#getting-started)
- [Authors](#authors)

---

## Overview

This project implements a dual-hazard (earthquake + fire) intelligent alert system on the ESP32 microcontroller. At its core, it runs an **on-device ML-inspired classification pipeline** that fuses multi-axis accelerometer data from the MPU6500 with ambient temperature readings from the DHT11 sensor. The model continuously extracts the horizontal acceleration magnitude feature vector, applies trained decision thresholds, and uses a **sliding window sample-voting mechanism** to classify seismic events in real time — eliminating noise and false positives that single-sample detection cannot handle.

Upon a confirmed hazard classification, the system autonomously triggers power cutoff via a relay, activates audio-visual alarms, and dispatches SMS alerts to pre-configured recipients through the CircuitDigest Cloud API.

---

## Features

- **On-Device ML Classification** — Runs a lightweight, real-time seismic event classifier directly on the ESP32 with no cloud inference needed
- **Feature Extraction Pipeline** — Computes horizontal acceleration magnitude (`√(ax² + ay²)`) as the primary feature vector at 5 Hz
- **Sliding Window Voting** — Requires 3 consecutive positive classifications before triggering an alarm, suppressing noise and false positives
- **Dual-Sensor Fusion** — Combines accelerometer (MPU6500) and temperature (DHT11) data for multi-class hazard detection (earthquake + fire)
- **Adaptive Recovery Classification** — Uses a 25-sample stable-window classifier to confirm the seismic event has ended before transitioning states
- **Automatic Power Cutoff** — Relay activates immediately upon confirmed earthquake classification
- **Post-Earthquake Fire Detection** — Secondary classifier monitors temperature during recovery; triggers fire lockdown if temperature exceeds 45°C
- **SMS Alerts** — Sends earthquake and fire alerts to multiple recipients via CircuitDigest API
- **OLED Status Display** — Real-time display of magnitude, threshold, classification counters, temperature, and humidity
- **RGB LED Indicators** — Color-coded status (Green = Monitoring, Blinking Red = Earthquake Detected, Blinking Blue = Recovery, Solid Red = Fire)
- **Buzzer Alarm** — Audible alert during active alarm state
- **WiFi Connectivity** — Required for SMS delivery

---

## Hardware Requirements

| Component | Model / Spec |
|---|---|
| Microcontroller | ESP32 (any 38-pin variant) |
| IMU / Accelerometer | MPU6500 (I2C) |
| Temperature & Humidity | DHT11 |
| Display | SSD1306 OLED 128×64 (I2C) |
| Buzzer | Active buzzer (5V) |
| Relay | 5V single-channel relay module |
| RGB LED | Common-cathode RGB LED |
| Resistors | As needed for LED current limiting |
| Power Supply | 5V USB or regulated supply |

---

## Wiring / Pin Configuration

### I2C Bus (SDA = GPIO 21, SCL = GPIO 22)
| Device | Address |
|---|---|
| MPU6500 | `0x68` |
| SSD1306 OLED | `0x3C` |

### GPIO Pin Map
| GPIO | Connected To |
|---|---|
| 13 | Buzzer |
| 19 | Relay |
| 21 | I2C SDA (MPU6500 + OLED) |
| 22 | I2C SCL (MPU6500 + OLED) |
| 23 | DHT11 Data |
| 25 | RGB LED — Red |
| 26 | RGB LED — Green |
| 27 | RGB LED — Blue |

---

## Software Dependencies

Install the following libraries via the Arduino Library Manager or PlatformIO:

| Library | Purpose |
|---|---|
| `WiFi.h` | ESP32 Wi-Fi (built-in) |
| `HTTPClient.h` | HTTP POST for SMS API (built-in) |
| `Adafruit_SSD1306` | OLED display driver |
| `ArduinoJson` | JSON payload for SMS API |
| `Wire.h` | I2C communication (built-in) |
| `MPU6500_WE` | MPU6500 accelerometer driver |
| `DHT11` | DHT11 temperature/humidity sensor |

---

## Configuration

Open the sketch and update the following constants before uploading:

```cpp
// Wi-Fi credentials
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// CircuitDigest SMS API
const char* SMS_API_KEY      = "YOUR_API_KEY";
const char* ALERT_MOBILES_CSV = "91XXXXXXXXXX,91YYYYYYYYYY"; // Comma-separated, with country code
```

### Tunable Thresholds

```cpp
#define EARTHQUAKE_MAGNITUDE_THRESHOLD  0.30  // Min horizontal 'g' to count as shaking
#define SIGNIFICANT_MOTION_THRESHOLD    0.25  // Below this, shaking is considered stopped
#define TRIGGER_SAMPLES                 3     // Consecutive readings needed to trigger alarm
#define RECOVERY_SAMPLES                25    // Consecutive stable readings to exit alarm (≈5s)
#define FIRE_TEMP_THRESHOLD             45.0  // °C — triggers fire lockdown during recovery
```

---

## ML Detection Pipeline

### Feature Extraction
The system extracts the **horizontal acceleration magnitude** as its primary feature from raw MPU6500 sensor readings:

```cpp
// Feature vector computation
currentMagnitudeH = sqrt(ax * ax + ay * ay);  // Euclidean norm of X-Y plane
```

### Classification Model
The classifier uses a **sliding window majority-vote** approach — a lightweight technique commonly used in embedded ML for time-series event detection:

| Parameter | Value | Purpose |
|---|---|---|
| Sampling Rate | 5 Hz (200ms) | Data acquisition frequency |
| Earthquake Feature Threshold | 0.30g | Minimum magnitude to classify as seismic |
| Motion Silence Threshold | 0.25g | Below this, sample is classified as "stable" |
| Trigger Window (Positive votes) | 3 samples | Consecutive positives needed to confirm earthquake |
| Recovery Window (Negative votes) | 25 samples | Consecutive stable samples to confirm event end (~5s) |
| Fire Feature Threshold | 45°C | Temperature threshold for fire classification |

### State Machine (Inference Output)
```
[MONITORING]  ──[3 consecutive magnitude ≥ 0.30g]──────▶  [EARTHQUAKE DETECTED]
                  (relay ON, buzzer ON, LED blinks red)

[EARTHQUAKE]  ──[25 consecutive magnitude < 0.25g]──────▶  [RECOVERY]
                  (buzzer OFF, LED blinks blue, SMS sent)

[RECOVERY]    ──[temperature ≥ 45°C]────────────────────▶  [FIRE LOCKDOWN]
                  (relay ON, buzzer ON solid, LED solid red, SMS sent)

[RECOVERY]    ──[10 seconds elapsed, no fire]────────────▶  [MONITORING]
                  (relay OFF, LED solid green)
```

### Why Sliding Window Voting?
Traditional single-sample threshold detection is highly susceptible to vibration noise from footsteps, doors, or mechanical interference. The sliding window approach mimics a **binary classifier with temporal smoothing** — only labeling an event as a true earthquake when enough consecutive evidence supports it. This significantly reduces the false positive rate in real-world deployments.

---

## System States

| State | LED Color | Buzzer | Relay | Display |
|---|---|---|---|---|
| Idle / Monitoring | Solid Green | Off | Off | Seismic Monitor |
| Earthquake Alarm | Blinking Red | Beeping | ON (Power Cut) | `! ALERT !` banner |
| Recovery Mode | Blinking Blue | Off | ON (Power Cut) | Recovery countdown |
| Fire Lockdown | Solid Red | ON continuously | ON (Power Cut) | `FIRE EMERGENCY` |

---

## SMS Alert System

SMS alerts are sent via the **CircuitDigest Cloud SMS API** using HTTP POST.

| Event | Template ID | Variables |
|---|---|---|
| Earthquake detected | `101` | `var1`: "Seismic Sensor", `var2`: magnitude in g |
| Fire detected | `102` | `var1`: "Emergency System", `var2`: temperature in °C |

Alerts are sent only once per event (guarded by `smsEarthquakeSent` and `smsFireSent` flags) to prevent duplicate messages.

---

## OLED Display Screens

| Screen | When Shown |
|---|---|
| Boot Screen | Startup — shows name, version, loading bar |
| Ready Screen | After initialization — confirms MPU6500, DHT11, SMS API status |
| Seismic Monitor | Normal monitoring — shows magnitude, threshold, shake/stable counters, temp, humidity |
| Alert Screen | During earthquake alarm — blinking `! ALERT !` header |
| Recovery Screen | Post-earthquake — shows countdown timer and "SMS Alert Sent" |
| Lockdown Screen | Fire emergency — shows title, evacuation message, power cutoff status |

---

## Serial Monitor Output

Set baud rate to **115200**. The serial output provides detailed logs including:

- Boot banner with hardware pin summary
- Sensor initialization status (MPU6500, DHT11, OLED)
- Wi-Fi connection progress and IP address
- MPU6500 horizontal magnitude readings (every 2 seconds)
- State transitions with timestamps
- SMS send attempts and API response codes

---

## Getting Started

1. Wire up all components as per the [pin map](#wiring--pin-configuration).
2. Install all required [libraries](#software-dependencies).
3. Update [Wi-Fi and SMS credentials](#configuration) in the sketch.
4. Upload to the ESP32 using Arduino IDE or PlatformIO.
5. Open Serial Monitor at 115200 baud to verify initialization.
6. The system will self-calibrate the MPU6500 on boot — keep the device still during startup.
7. Once "SYSTEM READY" appears on the OLED, the system is actively monitoring.

---
