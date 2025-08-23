# Future Engineers 2025 – Self-Driving Vehicle

## Introduction
This repository contains the source code and documentation for our team’s entry into the **World Robot Olympiad (WRO) 2025 – Future Engineers: Self-Driving Cars** category. The project showcases our engineering design process, code development, and vehicle implementation, in line with the competition requirements.

Our self-driving vehicle is designed to complete the **Open Challenge** and **Obstacle Challenge** fully autonomously, while meeting the engineering documentation standards specified in the [WRO 2025 General Rules](https://wro-association.org/competition/questions-answers/).

---

## Engineering Design Discussion

### Mobility Management
- **Chassis & Drive:**
  - Four-wheeled vehicle design with a single driving axle and steering actuator, compliant with WRO regulations.
  - DC motors are used for forward and reverse motion, controlled through a gearing system for stability.
  - Steering is managed by a servo motor, providing precise turning at intersections and curves.

- **Control Algorithms:**
  - Implemented a **PID controller** (`pid.py`) for smooth line following and curve negotiation.
  - `robot.py` integrates mobility functions, coordinating motor control with sensor feedback.

### Power and Sense Management
- **Power Supply:**
  - Powered by rechargeable Li-Ion batteries, ensuring lightweight design and stable voltage for sensors and motors.

- **Sensors:**
  - **Color sensor (`color.py`)**: Detects track lines and aids in position correction.
  - **Gyroscope (`gyro.py`)**: Provides orientation feedback, critical for precise turns.
  - **Pixy camera (`pixy.py`)**: Used for vision-based obstacle and sign detection.
  - **Additional modules (`fft.py`, `read.py`)**: Provide signal processing and sensor data handling from external boards (e.g., Raspberry Pi Pico).

- **Sensor Fusion:**
  - Sensor readings are fused to improve localization and decision-making in dynamic track conditions.

### Obstacle Management
- The **Obstacle Challenge** requires recognition and response to traffic signs (green and red pillars).
- Implemented in `pixy.py` with image processing:
  - **Red pillar** → keep right.
  - **Green pillar** → keep left.
- Collision avoidance and obstacle path adjustments are managed through a decision module in `main.py`.
- Vehicle parking is executed after completing laps, ensuring alignment and parallel positioning inside the parking lot.

---

## Repository Structure
```
future-engineers-2025/
│
├── src/
│   ├── ev3/
│   │   ├── color.py        # Color sensor handling
│   │   ├── gyro.py         # Gyroscope handling
│   │   ├── pid.py          # PID controller
│   │   ├── pixy.py         # Pixy camera integration
│   │   ├── robot.py        # Core robot control class
│   │   ├── usb.py          # USB communications
│   │   ├── util.py         # Utility functions
│   │   ├── main.py         # Main EV3 program entry point
│   │   └── test.py         # Test utilities for components
│   │
│   ├── pico/
│   │   ├── fft.py          # Signal processing (Pico)
│   │   ├── read.py         # Pico data reading
│   │   └── test.py         # Pico module tests
│   │
│   └── serial_ev3/
│       └── main.py         # EV3 serial communication entry point
│
└── README.md               # Documentation (this file)
```

---

## Building and Running the Code
### Requirements
- **Hardware:** LEGO EV3 brick, motors, Pixy camera, gyro sensor, Raspberry Pi Pico.
- **Software:** Python 3.8+, EV3 MicroPython runtime, serial communication support.

### Setup Instructions
1. Clone this repository:
   ```bash
   git clone https://github.com/<team-name>/future-engineers-2025.git
   cd future-engineers-2025/src/ev3
   ```
2. Deploy Python scripts to the EV3 brick.
3. Connect sensors and verify functionality using `test.py`.
4. Run the main program:
   ```bash
   python3 main.py
   ```

---

## Media and Demonstration
- **Vehicle Photos:** (to be inserted by team)
- **Team Photo:** (to be inserted by team)
- **Demonstration Videos:**
  - [Open Challenge Demo](https://youtube.com/...)  
  - [Obstacle Challenge Demo](https://youtube.com/...)

---

## GitHub Commit Policy
As per WRO documentation rules:
- ✅ **First commit:** at least 2 months before competition (≥20% of code).  
- ✅ **Second commit:** at least 1 month before competition.  
- ✅ **Third commit:** at least 2 weeks before competition.  
- Additional commits allowed and encouraged.

---

## Conclusion
This repository presents the complete engineering progress, codebase, and documentation of our autonomous vehicle. It reflects our efforts in **mobility control, power and sensing management, and obstacle navigation**, while following the **real engineering process** required by the WRO Future Engineers category.

---

## License
Open-source under the MIT License – reuse and improvements encouraged.

