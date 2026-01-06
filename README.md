# Autonomous Rover with Sensor-Based Navigation   

This project presents the design and implementation of a fully autonomous mobile rover using an Arduino Mega, developed as a systems-level exploration of embedded control, sensor integration, and real-time decision-making. The rover autonomously navigates its environment using rotating ultrasonic sensors for 360° obstacle detection, differential motor control for motion execution, and geometric kinematic modeling to estimate orientation and relative position. Emphasis is placed on hardware-aware algorithm design, power domain separation, and reliability under constrained computational resources, mirroring core challenges in real-world robotic and planetary exploration systems.

## Overview

This repository contains the design and implementation of a fully autonomous mobile rover built on an **Arduino Mega**, capable of **sensor-based navigation and real-time obstacle avoidance**. The rover operates without external localization, vision systems, or user input, relying instead on ultrasonic distance sensing, servo-driven environmental scanning, and embedded motion modeling.

The project was developed as a final course project for *PHY-315-A* at Davidson College and draws inspiration from autonomous planetary rovers such as NASA’s *Perseverance*, with an emphasis on **hardware-level system design**, **embedded control**, and **real-time physical interaction**.

---

## Key Features

- Fully autonomous navigation with no remote control
- Real-time obstacle detection using ultrasonic sensors
- 360° environmental scanning via rotating micro servo motors
- Reactive path optimization using geometric motion modeling
- Differential-drive control using TT DC motors and L298N motor drivers
- Onboard estimation of rover orientation and relative X–Y position
- Custom-designed and 3D-printed chassis with multi-domain power distribution

---

## System Architecture

### Hardware Platform
- **Microcontroller:** Arduino Mega  
- **Motors:** 4× TT DC motors (differential drive)  
- **Motor Drivers:** 2× L298N dual H-bridge drivers  
- **Sensors:** 3× Ultrasonic distance sensors  
- **Actuators:** 3× SG90 micro servo motors  
- **Power:** Isolated battery domains with LM7805 voltage regulation  
- **Chassis:** Custom 3D-printed (TinkerCAD design)

### Software Architecture
- Modular Arduino firmware
- Sensor acquisition and filtering
- Servo-driven scanning routines
- Kinematic modeling for orientation and displacement
- Reactive navigation algorithm with reservation / fulfillment phases

---

## Navigation Algorithm

The rover’s autonomous behavior follows a **two-stage reactive navigation loop**:

1. **Reservation Phase**  
   - Front-mounted ultrasonic sensor sweeps 45°–135°
   - Obstacle detected if distance < 25 cm
   - Rover brakes and halts motion

2. **Fulfillment Phase**  
   - Rear-mounted ultrasonic sensors perform a full 360° scan
   - Direction with maximum clearance is selected
   - Rover rotates toward the optimal heading
   - Heading change is estimated via wheel kinematics and elapsed time

Once aligned within ±5°, the rover resumes forward motion and repeats the cycle.

---

## Repository Structure

```text
autonomous-rover/
├── README.md
│
├── docs/
│   ├── report.pdf
│   ├── figures/
│   │   ├── circuit_diagram.png
│   │   ├── chassis_layers.png
│   │   └── test_environment.jpg
│   └── videos/
│       └── rover_demo.mp4
│
├── firmware/
│   ├── rover_main/
│   │   ├── rover_main.ino
│   │   ├── motor_control.h
│   │   ├── motor_control.cpp
│   │   ├── ultrasonic.h
│   │   ├── ultrasonic.cpp
│   │   ├── navigation.h
│   │   ├── navigation.cpp
│   │   └── config.h
│   └── README.md
│
├── hardware/
│   ├── stl/
│   │   ├── chassis_base.stl
│   │   ├── motor_mount.stl
│   │   ├── sensor_mount_front.stl
│   │   └── sensor_mount_rear.stl
│   ├── tinkercad/
│   │   └── rover_chassis_design.json
│   └── README.md
│
├── media/
│   ├── thumbnails/
│   └── gifs/
│       └── rover_navigation.gif
│
└── .gitignore
```