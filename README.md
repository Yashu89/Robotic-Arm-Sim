# ROS2 Robotic Arm Simulation

A ROS2 based robotic arm simulation that uses LiDAR and camera sensors to detect, locate, and pick-and-place objects. The project is built using Gazebo for simulation and MoveIt 2 for robotic motion planning and manipulation.

---

## Overview

This project implements a simulated robotic manipulation system capable of searching for an object, detecting its position, planning a motion trajectory, picking up the object, and placing it at a predefined basket.

The project integrates perception, localization, motion planning, and robotic manipulation into a complete pick-and-place workflow.

---

## Demonstration

The demonstration shows the complete workflow, including object search, detection, localization, fine centering using the wrist-mounted camera, object pickup, and placement into the basket.

[Watch the project demonstration](RoboticArm_Demo.mp4)

---

## System Architecture

The system follows a perception-to-manipulation pipeline:

```text
             Search Camera
                   │
                   ▼
      Object Detection & Alignment
                   │
                   ▼
              LiDAR Data
                   │
                   ▼
          Object Localization
                   │
                   ▼
             Wrist Camera
                   │
                   ▼
            Fine Centering
                   │
                   ▼
          MoveIt 2 (IK + Planning)
                   │
                   ▼
              Pick Object
                   │
                   ▼
             Place in Basket

```
---

## Hardware and Sensors

- **UR5 Robotic Arm** – Main manipulator used for pick-and-place operations.
- **Two-Finger Gripper** – Used to grasp and release the object.
- **Search Camera** – Detects the object and performs initial alignment.
- **LiDAR** – Provides distance information for object localization.
- **Wrist-Mounted Camera** – Used for fine centering and alignment during pickup.

---

## Software Technologies Used

- ROS 2 Jazzy
- Gazebo
- MoveIt 2
- URDF/Xacro
- Python
- ROS 2 Control
- OpenCV
- RViz

---

## Installation

```bash
git clone https://github.com/yashu8919/Robotic-Arm-Sim.git
cd Robotic-Arm-Sim

# Build
colcon build
source install/setup.bash
```

## Running the Simulation

**Terminal 1 - Launch Gazebo simulation:**
```bash
source install/setup.bash
ros2 launch robotic_arm bringup.launch.py
```

**Terminal 2 - Start pickup controller:**
```bash
source install/setup.bash
ros2 launch robotic_arm pickup.launch.py
```

---

## Key Components Implemented

- **Search Detector**: Camera based object search and alignment using HSV masking
- **Pickup Controller**: Controls the pick-and-place sequence using sensor inputs and MoveIt 2
- **Gripper Control**: Joint-based parallel gripper manipulation
- **Sensor Fusion**: Combines LiDAR and camera data for object localization and centering
- **Pick-and-Place Workflow**: End-to-end perception and manipulation pipeline

---

## Limitations

- The project is currently designed and tested in a simulated Gazebo environment.
- The maximum object size that can be reliably grasped depends on the gripper opening and positioning accuracy.
- Objects close to the maximum gripper opening may not be reliably grasped due to positioning and centering errors.
- Object height is limited by the robotic arm's reachable workspace and possible collision constraints.
- The basket/drop location is predefined.
- The system has primarily been tested with a specific object shape and size.

---

## Future Improvements

Possible future improvements include:

- Improving object localization accuracy
- Improving grasp planning
- Dynamic object detection and tracking
- Collision-aware manipulation improvements
- Support for additional object shapes and sizes

---

## Acknowledgments

- UR5 model from open-source robotics resources
- MoveIt 2 for motion planning framework
- ROS 2 and Gazebo communities

