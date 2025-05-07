# Adam-voice-controlled-robot
Fully autonomous voice controlled robot developed using ROS2, TurtleBot3 and NVIDIA Jetson


# ADAM - Voice Controlled Obstacle Avoidance Robot

An autonomous robot built using ROS 2 that responds to voice commands and avoids obstacles using LIDAR. Designed as a final year project and deployed on a TurtleBot3 Waffle Pi platform.

## 🧠 Features
- Voice control using Google Speech-to-Text API
- Obstacle detection and avoidance using LIDAR
- ROS 2 node architecture with modular design
- Simulated in Gazebo and deployed on real hardware

## 🛠 Tech Stack
- ROS 2 (Humble)
- Python
- Arduino
- Google STT API
- Gazebo Simulator
- LIDAR (RPLIDAR A1/A2)
- TurtleBot3 Waffle Pi
- Jetson Orin

## 🗂 Folder Structure
adam/
├── src/
│ ├── adam_closest_distance.py
│ └── adam_direction_distance.py
| └── adam_voice_control.py
│ 
├── docs/
│ └── poster.jpg
└── README.md

## Project Poster
![POSTER DRAFT](https://github.com/user-attachments/assets/64892a94-5402-4c81-af38-6bb178e75a7d)


## 🚀 Running the Nodes
```bash
colcon build
source install/setup.bash
ros2 run adam adam_closest_distance
ros2 run adam adam_direction_distance
ros2 run adam adam_voice_control
