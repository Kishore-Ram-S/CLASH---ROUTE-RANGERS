# CLASH Search & Rescue Bot (Route Rangers) 🤖🏆

![Arduino](https://img.shields.io/badge/Microcontroller-Arduino%20Nano-blue?logo=arduino)
![C++](https://img.shields.io/badge/Language-C%2B%2B-brightgreen?logo=c%2B%2B)
![Open Source](https://img.shields.io/badge/Open%20Source-Yes-success)

An **autonomous line follower + color-based pick & place rescue bot** built for the **CLASH event (Build Club SSN)**.  
The robot follows the line using QTR sensors, detects a box using an IR sensor, identifies the box color using the **TCS34725**, and picks it using our **custom PLA 3D-printed gripper**.

---

## 🚀 Features
- **Autonomous line following** using **2× QTR sensors**
- **Phase 1 Scan → Broken line detection → Mandatory U-Turn**
- **Instant box detection** using IR sensor (stops immediately)
- **Color recognition** using **TCS34725 RGB sensor**
- **Pick & Place** using **custom 3D-printed PLA gripper**
- **Smooth DC motor control** using **TB6612FNG**
- **Bluetooth support** using **HC-05** (debug/control)

---

## 🧠 Working Logic (Competition Flow)
1. **Scan Run (Phase 1):** Bot follows the track and ignores boxes until the broken-line scan point.
2. **Mandatory U-Turn:** After scan completion, the bot performs a **180° turn**.
3. **Search & Rescue (Phase 2):** Bot hunts for boxes, detects them using IR, scans color using TCS34725, and performs **instant grab**.
4. **Delivery:** Bot navigates to intersection and turns based on held box color:
   - **RED → Left Zone**
   - **BLUE → Right Zone**
   - **GREEN → Avoid (no pickup)**

---

## 🛠 Hardware Components

| Component | Qty | Description |
|----------|-----|-------------|
| **Arduino Nano** | 1 | Main microcontroller |
| **QTR IR Sensors** | 2 | Line detection sensors |
| **TCS34725 Color Sensor** | 1 | RGB color detection |
| **IR Sensor** | 1 | Box/object detection |
| **TB6612FNG Motor Driver** | 1 | Dual H-bridge driver |
| **N20 Motors** | 2 | Drive motors |
| **N20 Wheels** | 2 | Wheels for locomotion |
| **HC-05 Bluetooth Module** | 1 | Wireless communication |
| **Custom PLA Gripper** | 1 | 3D-printed gripper (designed by us) |

---

## ⚙️ Code Highlights
- **State Machine based navigation**
  - `NAV_TO_SCAN → SCAN_UTURN → HUNTING → DELIVERING → DROP_OFF`
- **PID line following** for stable tracking
- **Gap chain counting** for broken-line scan detection
- **Instant stop + scan + grab** mechanism for quick pickup

---

## 📚 Libraries Required
- **QTRSensors**
- **SparkFun TB6612FNG Motor Driver**
- **Adafruit TCS34725**
- **Servo**
- **Wire**

---

## 📌 Notes
- Servo angles and thresholds are tuned for our build (may need calibration for other setups).
- IR sensor output logic may vary depending on module type.

---
