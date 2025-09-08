# Autonomous RC Car with AI Vision

An autonomous 1:10 scale RC car powered by a **Raspberry Pi Zero 2 W**, combining **embedded systems, computer vision, and robotics**.  
The car can drive manually via an **RC controller**, or autonomously using **AI-based vision** (lane following, stop sign & traffic light detection).  
A **Pololu multiplexer** ensures safe switching between manual and autonomous modes.

---

## Hardware Components

### Core
- Raspberry Pi Zero 2 W (onboard computer)
- Pi Camera Module v3 (vision)
- 7" HDMI Display 1024x600 (UI / "face")
- MPU-6050 (IMU: accelerometer + gyroscope)
- VL53L1X (ToF distance sensor)

### Drive & Control
- Surpass Hobby 3650kV Brushless Motor
- QuicRun WP 10BL60 Brushless ESC (60A)
- Miuzei 25 kg 270° Digital Servo (steering)
- RadioLink RC4GS v3 Transmitter + R6FG Receiver
- Pololu 2806 Servo Multiplexer (manual/autonomous switching)

### Power
- 5200 mAh Zeee 80C 2S LiPo (7.4V, 38.38 Wh)
- DROK 2203 Adjustable Buck Converter (8–22V → 5V for Pi)
- Inline fuse (30–40 A) + Master power switch
- XT60/Deans connectors + 14–18 AWG silicone wire

---

## System Architecture

LiPo Battery (7.4 V)
 ├──> ESC (QuicRun 10BL60) → Brushless Motor
 │     └──> BEC 6 V → RadioLink Receiver + Servo (via Pololu)
 ├──> DROK Buck (5.2 V) → Raspberry Pi Zero 2 W
 │        ├──> Camera Module v3 (CSI ribbon)
 │        ├──> 7" HDMI Display (HDMI + USB for touch)
 │        ├──> MPU-6050 IMU (I²C)
 │        └──> VL53L1X ToF (I²C)
 └──> Inline Fuse + Master Switch (safety)

---

## Wiring Guide

### Power
- LiPo → ESC (main motor power)
- LiPo → DROK Buck → Pi (5.2 V regulated)
- ESC BEC → Receiver + Servo (through Pololu multiplexer)

### Control Signals (via Pololu 2806)
- **Receiver CH1 (Steering)** → Pololu IN A1
- **Receiver CH2 (Throttle)** → Pololu IN A2
- **Pi GPIO18 (Steering PWM)** → Pololu IN B1
- **Pi GPIO12 (Throttle PWM)** → Pololu IN B2
- **Pololu OUT1 → Servo** (steering)
- **Pololu OUT2 → ESC** (throttle)
- **Pololu Select Line** → spare channel on RadioLink (manual vs autonomous)

### Raspberry Pi GPIO Pinout

| Pi Pin # | GPIO # | Function     | Connected Device            |
|----------|--------|-------------|-----------------------------|
| 1        | 3.3V   | Power       | MPU-6050, VL53L1X           |
| 2 / 4    | 5V     | Power       | Pi, Display, Camera (via buck) |
| 3        | GPIO 2 | I²C SDA     | MPU-6050, VL53L1X           |
| 5        | GPIO 3 | I²C SCL     | MPU-6050, VL53L1X           |
| 6 / 9    | GND    | Ground      | Common ground (all devices) |
| 12       | GPIO18 | PWM (steer) | Pololu IN B1                |
| 32       | GPIO12 | PWM (throt) | Pololu IN B2                |
| CSI      | —      | Camera      | Pi Camera Module v3         |
| HDMI     | —      | Video Out   | 7" Display                  |

---

## Software Stack

- **OS**: Raspberry Pi OS Lite (Bullseye)
- **Control**: `pigpio` (hardware PWM) or `PCA9685` (optional driver)
- **Computer Vision**: OpenCV (lane detection, color segmentation)
- **AI Models**: TensorFlow Lite (SSD MobileNet / YOLOv8-nano INT8)
- **Sensor Drivers**:
  - `smbus2` for I²C sensors (MPU-6050, VL53L1X)
  - `picamera2` for camera input
- **UI**: Python + Pygame / Tkinter on 7" HDMI screen

---

## Basic Control Loop (pseudo-code)

```python
while True:
    frame = camera.get_frame()
    lane_offset = detect_lane(frame)
    detections = detect_objects(frame)

    # Default forward motion
    throttle = BASE_SPEED
    steer = pid_control(lane_offset)

    # Stop sign logic
    if 'stop_sign' in detections and close_enough(detections['stop_sign']):
        throttle = 0

    # Traffic light logic
    if 'traffic_light' in detections:
        if detections['traffic_light'] == 'red':
            throttle = 0

    # Obstacle detection
    distance = tof_sensor.read()
    if distance < SAFE_DIST:
        throttle = 0

    send_pwm(steer, throttle)
```

---

## Features
- Manual RC mode (RadioLink)  
- Autonomous mode (AI vision + sensors)  
- Lane following with PID control  
- Stop sign & traffic light detection  
- Obstacle avoidance (ToF sensor)  
- On-screen UI (speed, detections, “face”)  
- Safe manual override via Pololu multiplexer  

---

## Stretch Goals
- Multi-car platooning (Wi-Fi UDP broadcasting)  
- AprilTag waypoint navigation  
- Map building with ORB-SLAM2 (Jetson Nano upgrade)  
- Personality UI (animated eyes/mouth on 7" display)  

---

## Safety
- Always test first with wheels lifted.  
- Use a 30–40 A inline fuse on the LiPo.  
- Never leave LiPo charging unattended.  
- Ensure **common ground** across ESC, Pi, receiver, and sensors.  

---

## Demo Ideas
- Record lane-following indoors with tape on the floor.  
- Print stop signs/traffic lights for detection demo.  
- Show manual/autonomous switch live via RC transmitter.  
- Overlay bounding boxes on 7" screen while driving.  

---

## Author
Built by **Vishit Soni** — Computer Engineering Student, University of Alberta.  
Demonstrates embedded systems, robotics, and applied machine learning.
