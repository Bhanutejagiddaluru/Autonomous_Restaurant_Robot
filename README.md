# Autonomous Restaurant Robot

## Overview
This project implements an **Autonomous Restaurant Robot** that navigates in a restaurant environment, delivers food orders, and interacts with kitchen staff. The robot utilizes SLAM (Simultaneous Localization and Mapping), object detection, IR sensors, and a custom-built GUI for ordering. The robot waits at kitchen doors for a green signal, detects food via image recognition, and ensures seamless delivery of orders to the designated tables.

## Features
- **Autonomous Navigation:** The robot autonomously navigates through the restaurant using pre-built maps, avoiding obstacles in real time.
- **Food Detection and Delivery:** The robot can identify specific food orders and deliver them to customers after picking them up from designated kitchens.
- **IR Sensor Integration:** IR sensors detect food trays when placed inside the robot's container and confirm delivery.
- **Kitchen Door Interaction:** The robot waits for a signal at the kitchen door and proceeds only after a green signal.
- **4-bit Data Encoding:** The robot’s order GUI encodes all data into 4-bit packets, reducing memory usage and allowing the robot to easily decode the data at the desired location.
- **User-Friendly GUI:** A PyQt5-based GUI interface allows restaurant staff to place orders and monitor the robot’s status in real-time.

## Technologies Used
- **Robot Operating System (ROS2 Humble):** Handles robot communication, SLAM, and path planning.
- **TurtleBot3:** The main robotic platform for this project.
- **Python:** Programming language used for all custom scripts, navigation, and sensor integration.
- **IR Sensors:** Used to detect the presence of food in the robot's container.
- **Raspberry Pi:** The main processing unit for the robot.
- **SLAM (Simultaneous Localization and Mapping):** Used for mapping the restaurant and enabling autonomous navigation.
- **PyQt5:** GUI framework used for order placement and monitoring.
- **Image Detection:** Integrated camera system for recognizing food items at the kitchen.
- **Data Encoding:** 4-bit data encoding system implemented to reduce memory usage and simplify the robot's order-decoding process.

## System Requirements
- **PC with Ubuntu and ROS2 Humble Installed**
- **TurtleBot3 Robot**
- **Raspberry Pi (mounted on the TurtleBot3)**
- **IR Sensors**
- **Camera for Image Recognition**
- **Mobile Hotspot for Communication**

## Folder Structure

### On PC:
```
/Documents
  ├── maps/
  ├── ros2_ws/
  └── turtlebot3_ws/
```

- **maps/**: Stores saved maps for navigation.
- **ros2_ws/**: Workspace for custom ROS2 packages like `turtle_control` and `order_gui`.
- **turtlebot3_ws/**: Workspace containing all TurtleBot3-related ROS2 packages.

### On Robot (TurtleBot3):
```
/Desktop
  └── dev_ws/
```

- **dev_ws/**: Workspace for development packages, including IR sensors, camera integration, and other robot-related functionality.

## Getting Started

### 1. **Setup on Robot**
### On the Robot (connect Raspberry Pi of the robot to Monitor)

1. **Install ROS2 Humble on the robot** (use appropriate ROS2 installation guide):
    ```bash
    sudo apt update && sudo apt install ros-humble-desktop
    ```
2. **Set up environment**:
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
3. **Install Arduino Library for TurtleBot3**: 
    - Follow instructions at:
      https://emanual.robotis.com/docs/en/parts/controller/opencr10/
    - Set the board to TurtleBot3 firmware.
      
4. **TurtleBot3 Setup on Arduino:**
    Follow the steps in the TurtleBot3 OpenCR setup:
    - Download the required **OpenCR** libraries.
    - Use this preference URL in your Arduino IDE preferences:
    ```
    https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
    ```
    Upload the appropriate firmware to your TurtleBot3.

5. **Connect to the mobile hotspot**:
    ```bash
    ssh <user>@<robot_ip>
    ```
    
6. **System Setup**:
    ```bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_bringup robot.launch.py
    ```

### 2. **ROS2 Humble Installation on PC**
Follow these steps to install ROS2 Humble on your PC.  
- **Step 1**: Set locale.
- **Step 2**: Setup sources.
- **Step 3**: Install ROS2 packages (`ros-humble-desktop`).
  
Skip installing `ros-humble-base`.

Once installed, update your bash configuration:
```bash
gedit ~/.bashrc
```
Add this line to the file:
```bash
source /opt/ros/humble/setup.bash
```

### 3. **Creating and Using Maps (SLAM)**

- Clone the TurtleBot3 repository:
```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
cd turtlebot3_ws
colcon build
source install/setup.bash
```
- Launch SLAM:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
- Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### 4. **Navigation and Path Planning**

#### On the Robot:
- Connect to the robot:
```bash
ssh ubuntu@10.42.0.1
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

- Start IR Sensor Publisher:
```bash
cd ~/dev_ws
colcon build
source install/setup.bash
ros2 run robot_sensors ir_sensor_publisher
```

#### On the PC:
- Launch navigation:
```bash
cd ~/Documents/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/maps/map.yaml
```
- Control the robot via teleoperation:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### 5. **Custom Python Navigation Code**
Run your custom Python navigation script:
```bash
cd ~/Documents/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run turtle_control turtle_control
```

### 6. **Order GUI Setup**
To run the PyQt5 GUI for placing orders:
```bash
cd ~/Documents/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run order_gui order
```

### 7. **IR Sensor Testing**
To test IR sensors from the PC:
```bash
source install/setup.bash
ros2 run turtle_control ir_sensor_subscriber
```

## Common Issues and Fixes

### 1. **No Internet Connection When Using Robot Hotspot**
If you face internet issues while connected to the robot’s hotspot, try using Google's DNS:
```bash
sudo nano /etc/resolv.conf
```
Add the following lines:
```
nameserver 8.8.8.8
nameserver 8.8.4.4
```

### 2. **Camera Not Detected**
If you encounter issues with the camera not being detected:
1. Ensure the camera is connected to the correct port:
```bash
lsusb
```
Check for the device ID:
```
Logitech, Inc. Webcam C270
```
2. Restart the camera and re-run your script:
```bash
ros2 run sdr_vision camera_to_ros
```

### 3. **Random Errors in RVIZ**
If you receive random "Message Filter dropping message" errors, ensure that all transforms are properly set up and you are connected to the internet when necessary.

---

## Notes
- For more detailed step-by-step installation of TurtleBot3 for **ROS2 Humble**, check the following resources:
    - **GitHub Releases** for TurtleBot3:  
      https://github.com/ROBOTIS-GIT/turtlebot3/releases
    - **eManual for TurtleBot3 Simulation**:  
      https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

## Conclusion
The Autonomous Restaurant Robot project combines the power of ROS2, TurtleBot3, SLAM, and custom sensors to create a highly efficient and interactive solution for food delivery in restaurants. With proper system setup on the robot and PC, the project is scalable and modifiable for various automation tasks within the hospitality industry.
