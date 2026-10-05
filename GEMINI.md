# Receptionist Robot Workspace Overview (GEMINI.md)

Welcome to the **Receptionist Robot** project workspace. This workspace contains the ROS 2 Jazzy host packages (running on the Radxa Dragon Q6A) and is integrated with low-level ESP32-S3 motor control firmware.

---

## 1. System Architecture

The robot employs a split-processing architecture:
<!-- 1. **High-Level Processing (Radxa Dragon Q6A):** Runs ROS 2 Jazzy, handles SLAM mapping (SLAM Toolbox), path planning (Nav2), LiDAR scanning (RPLidar A1/A2/A3), and safety monitoring (Nav2 Collision Monitor). -->
1. **High-Level Processing (Radxa Dragon Q6A):** Runs ROS 2 Jazzy, handles SLAM mapping (SLAM Toolbox), path planning (Nav2), LiDAR scanning (HCLiDAR / Camsense), and safety monitoring (Nav2 Collision Monitor).
2. **Low-Level Control (ESP32-S3):** Runs native C/C++ firmware (ESP-IDF v5.x) with micro-ROS, interfacing directly with motor driver hardware, reading quadrature encoders, and running real-time PID wheel velocity control loops.

```mermaid
graph TD
    subgraph Radxa Dragon Q6A [Radxa Host - ROS 2 Jazzy]
        Nav2[Nav2 Stack]
        SLAM[SLAM Toolbox]
        %% LidarDriver[sllidar_ros2 Node]
        LidarDriver[hclidar_driver_ros2 Node]
        ColMon[Collision Monitor]
        MicroAgent[micro-ROS Serial Agent]
    end

    subgraph ESP32-S3 [MCU Firmware - micro-ROS]
        StateM[micro-ROS State Machine]
        PID[Dual-Wheel PID Control]
        LEDC[LEDC PWM Motor Driver]
        Enc[Quadrature Encoder Driver]
    end

    %% Lidar[RPLidar Sensor] -->|USB Serial @ 460800| LidarDriver
    Lidar[HCLiDAR Sensor] -->|USB Serial @ 115200| LidarDriver
    ColMon -->|/cmd_vel | MicroAgent
    MicroAgent -->|UART @ 921600| StateM
    StateM -->|Velocity Target| PID
    PID -->|PWM Signals| LEDC
    LEDC -->|Voltage| Motors[DC Motors]
    EncT[Encoders] -->|Interrupts| Enc
    Enc -->|Feedback| PID
    PID -->|/odom & /tf| StateM
    StateM -->|/imu (MPU9250)| MicroAgent
    StateM -->|micro-ROS Transport| MicroAgent
```

---

## 2. Topic & Frame Routing

| Topic | Message Type | Publisher | Subscriber | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Collision Monitor / Teleop | ESP32-S3 | Final safe movement commands. |
| `/cmd_vel_nav` | `geometry_msgs/msg/TwistStamped` | Nav2 Controller | ESP32-S3 | Autonomous navigation commands. |
| `/cmd_vel_smoothed`| `geometry_msgs/msg/Twist` | Velocity Smoother | Collision Monitor | Smoothed output from Nav2. |
| `/odom` | `nav_msgs/msg/Odometry` | ESP32-S3 | Nav2 / SLAM / EKF | Dual-wheel encoder-derived odometry (20Hz). |
| `/imu` | `sensor_msgs/msg/Imu` | ESP32-S3 | EKF (`robot_localization`) | MPU9250 6-DOF / 9-DOF IMU orientation & gyro rate (target 50Hz). |
| `/tf` | `tf2_msgs/msg/TFMessage` | ESP32-S3 / Robot State Pub | TF Tree | Coordinate transformations ($odom \rightarrow base\_link$, etc.). |
<!-- | `/scan` | `sensor_msgs/msg/LaserScan` | RPLidar Driver | SLAM / Nav2 | Laser range scans for obstacle avoidance. | -->
| `/scan` | `sensor_msgs/msg/LaserScan` | HCLiDAR Driver | SLAM / Nav2 | Laser range scans for obstacle avoidance. |

---

## 3. Host Workspace Packages (`src/`)

### 📂 `receptionist_robot_bringup`
Contains the launch infrastructure and central configuration files:
<!-- * **`launch/bringup.launch.py`:** Main entrypoint that starts the micro-ROS agent, RPLidar driver, Robot State Publisher, SLAM Toolbox (mapping mode), and Nav2. -->
* **`launch/bringup.launch.py`:** Main entrypoint that starts the micro-ROS agent, HCLiDAR driver, Robot State Publisher, SLAM Toolbox (mapping mode), and Nav2.
* **`config/nav2_params.yaml`:** Configuration for costmaps, planner, and local controller.
  * *Global Planner:* `GridBased` (NavfnPlanner).
  * *Local Planner:* `FollowPath` (`dwb_core::DWBLocalPlanner`).
  * *Costmaps:* Configured with a `0.25m` robot radius. Global inflation is set to `0.6m` to prevent path cutting, and local inflation is set to `0.3m`.
* **`config/slam_toolbox_params.yaml`:** Configuration for 2D LiDAR SLAM mapping.

### 📂 `receptionist_robot_description`
Contains physical modeling and visualization components:
* **`urdf/receptionist_robot.urdf.xacro`:** URDF definition of the robot's physical structure, wheel placement, and sensor positions.
* **`rviz/receptionist_robot.rviz`:** Preconfigured RViz visualization profile.

<!-- ### 📂 `sllidar_ros2` -->
<!-- The ROS 2 driver package for the RPLidar sensor, configured to publish on `/scan` relative to the `laser` coordinate frame. -->
### 📂 `hclidar_driver_ros2`
The ROS 2 driver package for the HCLiDAR / Camsense sensor, configured to publish on `/scan` relative to the `laser` coordinate frame.

---

## 4. ESP32-S3 MCU Firmware Summary

* **Execution Rate:** **20Hz** (50ms cycles for motor/odom), **50Hz** (20ms target for IMU).
* **Communication Interface:** UART Serial @ **921600 baud** (micro-ROS Jazzy).
* **Communication Lifecycle:**
  * Auto-recovers and reboots (`esp_restart()`) to re-handshake and synchronize encoder counts if the Radxa micro-ROS agent restarts.
  * Embedded **watchdog** halts motors if no command is received on `/cmd_vel` or `/cmd_vel_nav` within **150ms**.
* **Sensors:**
  * **Encoders:** Quadrature optical/magnetic encoders on GPIO 7, 15 (Left) and 11, 16 (Right).
  * **IMU:** MPU9250 on I2C bus (SDA/SCL) reporting acceleration, angular velocity, and orientation.
* **Motor Control Specs:**
  * Uses **LEDC** at **2kHz** with **8-bit** duty cycle resolution (`0-255`).
  * Left motor pins: ENA `4`, IN1 `5`, IN2 `6`. Right motor pins: ENB `8`, IN3 `10`, IN4 `9`.
  * Left encoders: `7` & `15`. Right encoders: `11` & `16`.
  * Physical parameters: Wheel diameter = `65mm`, Wheel base = `300mm`, PPR = `325.0f`.
* **Kinematics & Control:**
  * Dual independent wheel PID loops ($K_P=100.0f$, $K_I=80.0f$, $K_D=1.2f$) with feedforward control.
  * Static deadband compensation (`90/255` PWM offset) to overcome gearbox/motor starting resistance.
  * Clamps minimum angular command to `0.25` rad/s to prevent low-velocity turning stalls.

---


