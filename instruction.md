# How to Run Scan, Map, and Navigation

This guide explains how to start your receptionist robot, build a map of your environment, save the map, and perform autonomous navigation.

---

## 1. Prerequisites & Setup

Ensure all hardware (ESP32-S3, RPLidar) is connected to the Radxa board.

1. **Verify Serial Device Connections**:
   - Check where the ESP32 is mounted: `ls /dev/ttyACM*` or `ls /dev/ttyUSB*` (typically `/dev/ttyACM0`).
   - Check where the RPLidar is mounted: `ls /dev/ttyUSB*` (typically `/dev/ttyUSB0`).
   - Grant read/write permissions to these ports if needed:
     ```bash
     sudo chmod a+rw /dev/ttyACM0
     sudo chmod a+rw /dev/ttyUSB0
     ```

2. **Source the ROS 2 Workspace**:
   Open a terminal on your Radxa board, navigate to the workspace, and source it:
   ```bash
   cd ~/receptionist_robot_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

---

## 2. Step 1: Launch the Robot Stack (Mapping Mode)

Launch the unified bringup file to start all nodes (Micro-ROS Agent, RPLidar driver, Robot State Publisher, SLAM Toolbox in mapping mode, and Nav2):

```bash
ros2 launch receptionist_robot_bringup bringup.launch.py \
  serial_port:=/dev/ttyACM0 \
  lidar_port:=/dev/ttyUSB0
```

> [!NOTE]
> Adjust the `serial_port` and `lidar_port` arguments if your devices are mounted on different ports.
> Once launched, the terminal will show that the Nav2 lifecycle manager has successfully brought up all servers, and the Micro-ROS Agent is connected to the ESP32.

---

## 3. Step 2: Teleoperate the Robot to Map the Room

While the bringup launch is running, open a **new terminal**, source the workspace, and run a teleoperation node to drive the robot around.

### Option A: Using teleop_twist_keyboard (Standard)
Install the teleop keyboard package if not already installed:
```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```
To run teleop while maintaining safety (routed through the Nav2 collision monitor):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_smoothed
```
*Alternatively, to drive directly without safety checks:*
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

```

### Option B: Monitor in RViz2 from a Remote PC
Since you are controlling the Radxa board via SSH, you should run RViz2 on your **Remote PC** (which has a monitor) to view the map and robot model in real-time.

1. **Network Configuration**:
   - Ensure both the Radxa board and your Remote PC are connected to the same Wi-Fi/local network.
   - Verify that they can ping each other.
   - Make sure they share the same `ROS_DOMAIN_ID` environment variable (default is `0`):
     ```bash
     echo $ROS_DOMAIN_ID
     ```

2. **Setup the Workspace on the Remote PC**:
   To visualize the 3D meshes of your robot correctly, copy or clone the `receptionist_robot_description` package to a workspace on your Remote PC, compile it, and source it:
   ```bash
   # On your Remote PC:
   cd ~/my_remote_ws
   colcon build --packages-select receptionist_robot_description
   source install/setup.bash
   ```

3. **Launch RViz2 on the Remote PC**:
   Run the pre-configured visualizer:
   ```bash
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix receptionist_robot_description)/share/receptionist_robot_description/rviz/receptionist_robot.rviz
   ```
   - If the packages aren't built on the Remote PC, you can launch a blank RViz (`ros2 run rviz2 rviz2`) and manually add display modules for:
     - **Map** (topic: `/map`)
     - **LaserScan** (topic: `/scan`)
     - **RobotModel** (description topic: `/robot_description`)
     - **TF** (displays coordinate frames)
   - Set the **Fixed Frame** in RViz to `map` (or `odom` if the map is not yet generated).

4. **Drive the Robot**:
   While watching the map update on your Remote PC, use the teleoperation terminal on the Radxa (via SSH) to drive the robot slowly around the room to clean up map boundaries and close loops.

---

## 4. Step 3: Save the Generated Map

Once you are satisfied with the map shown in RViz, you can save it.

### Method A: Save as standard Map Server files (YAML + PGM)
To save the occupancy grid for standard Nav2 navigation:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map
```
This will generate two files in your home directory:
- `my_robot_map.pgm` (the map image)
- `my_robot_map.yaml` (the map metadata configuration)

### Method B: Serialize Map via SLAM Toolbox Service
If you want to save the serialized map representation to continue editing it with SLAM Toolbox later:
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: 'my_serialized_map'}"
```

---

## 5. Step 4: Run Autonomous Navigation (Localization Mode)

To navigate autonomously using the saved map, you can load it in the navigation stack.

### Method 1: Using Nav2 Map Server + AMCL (Recommended)
1. Stop the active mapping process (`Ctrl+C` in all terminals).
2. **Launch the Hardware Drivers (Terminal 1)**:
   You must start the low-level micro-ROS agent, RPLidar, and Robot State Publisher so that transforms (`odom` and sensor links) are active:
   ```bash
   ros2 launch receptionist_robot_bringup hardware.launch.py \
     serial_port:=/dev/ttyACM0 \
     lidar_port:=/dev/ttyUSB0
   ```
3. **Launch Nav2 Localization (Terminal 2)**:
   In a second terminal, start the AMCL localization, map server, and path planners with your saved map:
   ```bash
   ros2 launch nav2_bringup bringup_launch.py \
     use_sim_time:=false \
     map:=/home/radxa/receptionist_robot_ws/map/test_map.yaml \
     params_file:=$(ros2 pkg prefix receptionist_robot_bringup)/share/receptionist_robot_bringup/config/nav2_params.yaml
   ```
4. In RViz on your Remote PC, set the robot's initial position using the **"2D Pose Estimate"** tool to align the laser scan with the loaded map.
5. Command the robot by using the **"2D Goal Pose"** tool in RViz to select a target destination.

### Method 2: Active SLAM Localization (SLAM Toolbox)
Alternatively, you can run SLAM Toolbox in localization mode to continuously localize against a saved pose graph:
1. Open the [slam_toolbox_params.yaml](file:///home/radxa/receptionist_robot_ws/src/receptionist_robot_bringup/config/slam_toolbox_params.yaml) file.
2. Change the following parameters:
   ```yaml
   mode: localization
   map_file_name: /home/radxa/my_serialized_map
   ```
3. Restart the bringup launch file:
   ```bash
   ros2 launch receptionist_robot_bringup bringup.launch.py
   ```
4. Nav2 will consume the map and TF updates dynamically published by SLAM Toolbox in localization mode.
