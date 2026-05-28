# Autonomous Mobile Robot (AMR) - Obstacle Avoidance using ROS2

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-4-red.svg)](https://www.raspberrypi.com/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)

## Overview

This project implements an **Autonomous Mobile Robot (AMR)** capable of detecting and avoiding obstacles using an ultrasonic sensor. The robot is built on a **Raspberry Pi 4** and uses **ROS2 (Robot Operating System 2)** for node communication and service-based architecture.

### Key Features
- Real-time obstacle detection using HC-SR04 Ultrasonic Sensor
- ROS2 service-based client-server architecture
- Autonomous navigation with obstacle avoidance logic
- Keyboard control mode for manual operation
- PWM motor control for precise movement

## Aim & Objectives

- Gain practical experience in building low-cost AMR systems
- Implement obstacle avoidance using ROS2 services and nodes
- Demonstrate successful communication between ROS2 client and service nodes
- Test and validate robot performance in real-world scenarios

## Hardware Components

| Component | Role |
|-----------|------|
| Raspberry Pi 4 | Main processing unit running ROS2 |
| HC-SR04 Ultrasonic Sensor | Distance measurement and obstacle detection |
| 2x DC Motors | Movement and direction control |
| 2x Wheels | Robot mobility |
| 4x 1.5V Batteries | Power supply for motors |
| Power Bank | Power supply for Raspberry Pi |
| Breadboard & Wires | Circuit connections |
| Chassis | Structural frame |


## Installation & Setup

### Prerequisites

- Raspberry Pi 4 with ROS2 Humble installed
- Python 3.10+
- GPIO access enabled

### Clone the Repository

```bash
# On Raspberry Pi
cd ~/ros2_ws/src
git clone https://github.com/yourusername/pirobot-ros2-obstacle-avoidance.git
cd ~/ros2_ws
colcon build --packages-select pirobot
source install/setup.bash
