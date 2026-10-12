# 🤖 ROS 2 Autonomous Visual Target Tracking & Obstacle Avoidance Robot

![ROS 2](https://img.shields.io/badge/ROS_2-Humble%2FIron%2FJazzy-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange)

An autonomous differential-drive robot simulation built with **ROS 2, Gazebo, C++, and OpenCV**.

The robot tracks a dynamically moving red target using computer vision while simultaneously navigating around obstacles detected by a 2D LiDAR sensor. Its behavior is controlled by a **Finite State Machine (FSM)** with proportional control, spatial memory, and obstacle avoidance.

---

## 🚀 Key Features

- 🎯 **Visual Target Tracking:** Detects and tracks a red target in real time using HSV color segmentation, contour filtering, and image moment analysis with OpenCV.
- 🧠 **Finite State Machine (FSM):** Implements structured behavior control for autonomous target tracking and obstacle avoidance.
- 🎮 **Proportional Control:** Adjusts linear and angular velocities according to the target's distance and position in the camera image.
- 🧭 **Spatial Memory:** Remembers the last known target direction, allowing the robot to reacquire a lost target without performing a full 360° search.
- 📡 **LiDAR Obstacle Avoidance:** Uses 2D LiDAR measurements to detect obstacles and navigate around them.
- ⚙️ **Deadband-Based Steering:** Reduces steering jitter and oscillations when avoiding obstacles directly ahead.
- 🏗 **Custom Robot Simulation:** Uses a differential-drive robot model defined in URDF.
- 📷 **Real-Time Visualization:** Publishes processed camera images for debugging and visualization through `rqt_image_view`.

---

## 📁 Repository Structure

```text
.
├── my_robot_controller/
│   ├── include/my_robot_controller # C++ header files
|   |   ├── vision_node.hpp
│   │   └── behavior_node.hpp     
│   ├── src/
│   │   ├── vision_node.cpp       # Visual target detection
│   │   └── behavior_node.cpp     # FSM and robot control
│   └── config/                   
│       └── vision_params.yaml    # ROS 2 parameter configuration
│
└── my_robot_description/
    ├── launch/                   
    │      └── sim.launch.py # Gazebo simulation launch files
    ├── urdf/                     
    │      └── robot.urdf # Robot model and Gazebo plugins
    └── worlds/                   
           └── test_world.world # Custom 8 × 6 m simulation arena
    
```

---

## 🛠️ Requirements

- Ubuntu with a compatible ROS 2 installation
- ROS 2 Humble, Iron, or Jazzy
- C++17
- Gazebo with ROS 2 integration
- OpenCV 4.x

### ROS 2 Dependencies

- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
- `robot_state_publisher`
- `gazebo_ros_pkgs`

---

## ⚙️ Installation & Build

### 1. Clone the Repository

Create a ROS 2 workspace and clone the project into its `src` directory.

```bash
cd ~/ros2_ws/src

git clone https://github.com/atykymschwep/robot_tracker_ros2.git
```

### 2. Install Dependencies

```bash
cd ~/ros2_ws

rosdep update

rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
colcon build --symlink-install

source install/setup.bash
```

---

## 🎮 Usage

### 1. Launch the Simulation

Start the Gazebo environment, robot model, perception node, and behavior controller:

```bash
ros2 launch my_robot_description robot_sim.launch.py
```

The robot will spawn in the simulated environment and begin processing camera and LiDAR data according to its behavior logic.

### 2. Visualize the Camera Feed

To inspect the raw camera image or the processed target-tracking output, launch `rqt_image_view` in a separate terminal:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select one of the following topics in the interface:

```text
/vision_tracker/image_processed
/camera/image_raw
```

The processed image allows you to observe the computer vision pipeline and verify target detection in real time.

---

## 🧠 System Architecture

The system consists of two primary ROS 2 packages working together.

### 1. Perception — `my_robot_controller`

The perception component processes camera images using OpenCV.

Its main tasks are:

1. Receive camera images through ROS 2.
2. Convert images to the HSV color space.
3. Segment the red target using color thresholds.
4. Filter contours to identify the target.
5. Calculate the target's image center using image moments.
6. Publish the processed image for visualization.

The processed image and target information provide the input for the robot's behavior controller.

### 2. Behavior Control — Finite State Machine

The behavior controller combines visual target information with LiDAR measurements to determine the robot's motion.

Its main responsibilities include:

- Tracking the target using proportional control.
- Adjusting linear and angular velocities.
- Remembering the last known target direction.
- Searching for a target that temporarily disappears from view.
- Detecting obstacles using LiDAR data.
- Applying deadband logic to reduce steering oscillations.

### 3. Simulation — `my_robot_description`

The simulation package provides the robot's URDF model, Gazebo configuration, launch files, and custom environment.

The robot uses a differential-drive configuration to move through the simulated arena while interacting with the target and surrounding obstacles.

---

## 📚 Tech Stack

| Category | Technology |
|----------|------------|
| Programming Language | C++17 |
| Robotics Middleware | ROS 2 |
| Simulation | Gazebo |
| Computer Vision | OpenCV 4.x |
| Image Integration | `cv_bridge` |
| Robot Description | URDF |
| Robot Control | Finite State Machine (FSM) |
| Control Method | Proportional Control |
| Obstacle Detection | 2D LiDAR |
| Visualization | `rqt_image_view` |
| Build System | CMake, colcon |

---

## 📷 Example Output


