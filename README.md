# Goku ROS 2 Package

A ROS 2 Jazzy package for a 4-wheel skid-steer robot with real hardware, SLAM, and navigation.

## Hardware
- Raspberry Pi 5: Hosts ROS 2, LiDAR (e.g., RPLidar), and camera.
- STM32F4: Controls motors, reads encoders and IMU (e.g., MPU6050).
- Sensors: LiDAR, camera, IMU, quadrature encoders.

## Setup
1. Install ROS 2 Jazzy and dependencies:
   ```bash
   sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-robot-localization libserial-dev
