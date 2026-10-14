# Decentralized Swarm Control

**Self-Regulated Multi-Agent Motion Algorithm using Bloch-type Perceptual Memory**

---

### Author
**Namrata Kumari**

### Guide
**Prof. Dr. Sohom Chakrabarty**  
Department of Electrical Engineering  
Indian Institute of Technology Roorkee

### Year
2026

---

## Project Overview

This project implements a **self-regulated multi-agent motion algorithm** for coordinated platoon behaviour of mobile robots. The algorithm is based on a two-timescale internal architecture consisting of:

- Fast Bloch-type perceptual registers
- Slow regulatory state

The system enables agents to achieve:
- Collision avoidance
- Group cohesion
- Directional alignment
- Adaptive velocity control
- Obstacle avoidance

The project includes both a complete **Python simulation** and a **micro-ROS ready C++ implementation** for ESP32-based mobile robots.

---

## Features

- Fast Bloch perceptual dynamics (`mx`, `my`, `mz`)
- Slow regulatory state dynamics
- Adaptive velocity based on terrain and cohesion risk
- Local neighbour detection using perception radius
- Obstacle avoidance
- Bounded heading control
- Differential drive ready structure for real robots

---
## Project Structure
Decentralized-Swarm-Control/
├── README.md
├── simulation/
│   └── swarm_simulation.py          # Complete Python simulation
└── micro_ros_agent/
├── platformio.ini
├── include/
│   └── swarm_controller.hpp     # Core algorithm (C++)
└── src/
└── swarm_agent.cpp          # ESP32 main program
---

## How to Run the Simulation

```bash
cd simulation
python3 swarm_simulation.py
Requirements:

Python 3
numpy
matplotlib

Current Status

Python Simulation: Completed and Working
Core Algorithm: Completed
ESP32 C++ Code: Completed
Code Compilation: Successful
Hardware Testing: Pending (due to ESP32 board detection issue)

Future Work

Multi-robot communication
Real-world testing on differential drive robots
Integration with ultrasonic sensors













