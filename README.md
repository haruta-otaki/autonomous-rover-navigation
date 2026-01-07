# Autonomous Rover with Sensor-Based Navigation   

This project explores the design of autonomous, spacecraft-inspired robotic systems through the development of a self-navigating rover. Inspired by planetary exploration rovers (e.g. NASA’s *Perseverance*), the system focuses on autonomous navigation, obstacle detection, and decision-making in uncertain environments. The rover operates by continuously scanning its surroundings using rotating ultrasonic sensors, evaluating alternative paths and selects a direction with minimal perceived interference when an obstacle is detected.

## Overview

This project presents the design and implementation of an autonomous rover capable of sensor-driven navigation and obstacle avoidance. The system integrates ultrasonic distance sensors, micro servo motors, DC drive motors, and motor drivers controlled by an Arduino Mega to perceive its environment, detect obstructions, and dynamically adjust its trajectory toward regions of minimal interference.

Autonomous decision-making is achieved through the application of fundamental principles in motion control, signal timing, and geometric interpretation. In addition to software design, careful consideration was given to power distribution and mechanical constraints, including the use of a 3D-printed chassis and the separation of power domains between logic and actuation components to ensure stable operation.

---

## System Modeling and Software Architecture

The rover interprets physical sensor data using unit conversions, geometric modeling, and trigonometric calculations to estimate its orientation and position. All quantities are computed relative to the rover’s initial position; hence, the initial X and Y coordinates, as well as the heading angle relative to the Y-axis, are initialized to zero.

### Orientation and Position Estimation

The rover estimates its orientation and position using wheel kinematics and elapsed time, without relying on external localization systems.

Servo angles are read directly in degrees; however, the rover’s global heading cannot be measured directly and must be estimated using wheel kinematics and elapsed time.

The heading is estimated from:
  - The linear velocity of the wheels
  - The elapsed time spent rotating
  - The horizontal separation between the wheels

Wheel linear velocity (v) is computed from the wheel diameter (65 mm) and motor speed (RPM):

v =  PI * 65 * (RPM/60).

Angular velocity (w) of the rover's turn is derived from differential wheel motion and wheel separation (126 mm):

w = ((2 * v) / 126) * (180/PI) 

Distance (d) the rover traveled over a control interval (t) is estimated as:

d = v * t

Therefore, the incremental kinematic update of the heading (theta) is defined as: 

theta_{t+1} = theta_t + w * t

where the position is updated in the 2D plane (x, y) using standard planar motion equations:

x_{t+1} = x_t + d * cos(theta_{t+1})
y_{t+1} = y_t + d * sin(theta_{t+1})

The incremental kinematic update loop enables continuous estimation of the rover’s orientation and relative X–Y position during autonomous operation.

---

## Navigation Algorithm

The rover’s autonomous navigation logic is structured around a **two-stage reactive path optimization process** consisting of a *reservation phase* and a *fulfillment phase*.

1. **Reservation Phase**  
   - Front-mounted ultrasonic sensor sweeps 45°–135°
   - Flags when obstacle is detected within 25 cm 
   - Rover brakes and transitions to fulfillment phase

2. **Fulfillment Phase**  
   - Rear-mounted ultrasonic sensors perform a full 360° scan
   - Direction with maximum clearance is returned
   - Rover rotates toward the optimal rotation angle
   - Heading change is estimated via wheel kinematics and elapsed time
   - Flags when rover rotation aligns within ±5° of the optimal heading
   - Rover halts left-turn maneuver andresumes forward motion 

---

## Power Distribution and Hardware Considerations

The rover employs multiple isolated power domains to ensure reliable operation and prevent electrical instability. Four batteries are used to independently supply power to the Arduino, the front and rear L298N motor drivers, and the servo motors.

Each pair of TT DC motors connected to an L298N motor driver requires a continuous 6 V supply. The isolation of the motor power from the Arduino prevents voltage drops and noise. Servo motors present an additional constraint: under load, they draw up to 650 mA, exceeding the Arduino’s maximum current output of approximately 500 mA. Furthermore, servo motors are not designed to accept a 9 V input, as excessive voltage can lead to overheating or permanent damage; hence, an LM7805 voltage regulator is used to step down the 9 V battery input to a stable 5 V supply dedicated to the servo motors.

---

## Repository Structure

```text
autonomous-rover/
├── README.md
│
├── docs/
│   ├── figures/
│      ├── circuit_diagram.png  # circuit diagram of the autonomous rover
│      ├── chassis_layers.png   # configurations of the 3D Print
│      └── test_environment.jpg # testing environment for the rover
│
├── firmware/
│   ├── rover_automation.i/
│   │   ├── rover_automation.i.ino
│
├── hardware/
    ├── Rover_Chassis_Bottom_Layer.stl
    ├── Rover_Chassis_Bottom_Layer.stl
```

## Results

The performance of the autonomous rover was primarily evaluated through video recordings of its operation, as autonomous navigation and real-time path optimization are not easily captured using scalar performance metrics. A full demonstration of the rover’s behavior may be viewed in the accompanying YouTube video:

**▶️ Demonstration Video:** *(link to be added)*

Testing was conducted within a confined artificial environment bounded by four walls and populated with randomly placed obstacles. To ensure consistency and fairness during evaluation, only obstacles with uniform dimensions were used. This constraint was necessary because the ultrasonic sensors operate at a fixed height; obstacles with irregular or non-uniform geometries fall outside the design assumptions of the sensing system and could lead to inaccurate or misleading distance measurements.

---

## Discussion

The rover consistently and successfully navigated the test environment across multiple trials. Its behavior closely matched the designed navigation algorithm, repeatedly executing the expected sequence of halting upon obstacle detection, activating the rear-mounted scanning sensors, rotating toward the direction of maximum perceived clearance, and resuming forward motion.

The rover also demonstrated robust performance in challenging scenarios, including navigation near corners and situations involving multiple surrounding obstacles. The consistent success observed across trials also reflects the effectiveness of the rover’s power distribution and mechanical design as no instances of component overheating, electrical malfunction, or structural instability were observed during testing. 

## Future Improvements

The autonomous rover developed in this project does not actively use the computed real-time X and Y position coordinates due to the absence of a wireless transceiver as well as the absense of higher-level behaviors to leverage the positional data. 

### Wireless Communication and Remote Control
A natural next step for this project is the integration of a **LoRa communication module**, a long-range, low-power radio technology designed for transmitting small data packets over extended distances. The enhancement would enable remote telemetry, allowing real-time transmission of position and sensor data without a physical connection. and support a manual control mode, where the rover may be operated through user inputs.

### Improved Motion Accuracy
The accuracy of the rover’s motion and sensor-derived data could be significantly improved through the addition of **wheel encoders**. The current system estimates movement using theoretical wheel RPM values, which may deviate due to surface friction, slippage, and mechanical inefficiencies. Wheel encoders would provide direct feedback on wheel rotation, enabling more precise distance estimation.

