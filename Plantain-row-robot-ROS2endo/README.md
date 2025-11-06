# Reactive Fuzzy Row-Following Robot for Plantain Crops

[![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)]()
[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-22314E.svg)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)]()

This repository contains code and data for a **low-cost tracked robot** that performs **reactive, fuzzy logic–based row following** in plantain plantations. The stack combines **ROS 2 (Raspberry Pi 5)** for perception/supervision and **ESP32** for low-level motor control (PI + dead-zone compensation).

> Paper title: **Reactive Fuzzy Control and Field Validation of a Low-Cost Autonomous Tracked Robot for Plantain Crop Row Following**  
> Keywords: agricultural robotics, LiDAR, fuzzy control, plantain crops, ROS 2, ESP32

---

## 📸 Field Pictures
<p align="center">
  <img src="docs/img/robot_field_side.jpg" alt="Tracked robot in plantain field" width="45%"/>
  <img src="docs/img/robot_field_front.jpg" alt="Front view under canopy" width="45%"/>
</p>

---

## 🧭 Features
- 2D LiDAR (YDLidar X2) + IMU (BNO055) for **lateral distance** and **yaw supervision**
- **Fuzzy wall follower** (Mamdani) with 5 input labels and 3 output labels per wheel
- **ESP32** low-level speed control (PI, anti-windup, dead-zone compensation, PWM 12-bit)
- ROS 2 nodes: `lidar_left`, `wall_follower`, `serial_bridge`
- Field-tested under **dense canopy** and **muddy terrain** in plantain crops

---

## 📂 Repository Layout
