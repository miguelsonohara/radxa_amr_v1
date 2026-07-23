# ESP32-S3 micro-ROS Robot Controller (Firmware Overview)

This workspace contains the C/C++ firmware (using ESP-IDF v5.x) running on an ESP32-S3. It connects to a Radxa Q6a over a custom UART serial transport to execute differential-drive robot commands and publish odometry feedback.

---

## 1. System Architecture & Communication

* **Transport:** Serial UART on `UART_NUM_0` configured via a custom transport layer (`esp32_serial_transport.c`).
* **Protocol:** micro-ROS (ROS 2 Jazzy).
* **Baud Rate:** `115200` baud.
* **Topics Subscribed:**
  * `/cmd_vel` (`geometry_msgs/msg/TwistStamped`) - Teleop/manual control commands.
  * `/cmd_vel_nav` (`geometry_msgs/msg/TwistStamped`) - Autonomous navigation commands.
* **Topics Published:**
  * `/odom` (`nav_msgs/msg/Odometry`) - Odometry containing pose, linear velocity, and angular velocity.
  * `/tf` (`tf2_msgs/msg/TFMessage`) - Coordinate frame transforms ($odom \rightarrow base\_link$).

---

## 2. Key Components & Files

### A. Main Coordinator: `main/hello_world_main.c`

* **`micro_ros_task`:** A state machine that manages the connection lifecycle with the Radxa agent:
  * **`STATE_WAITING_AGENT`:** Pings the agent at 115200 baud.
  * **`STATE_INITIALIZING`:** Prepares nodes, executors, subscriptions, and publishers.
  * **`STATE_PUBLISHING`:** Spins the executor for callbacks and publishes odometry at **20Hz** (50ms).
  * **`STATE_DEINITIALIZING` / Auto-Reconnection:** If a ping fails (agent disconnected or restarted), the ESP32 automatically stops the motors and reboots via `esp_restart()`. This guarantees a clean reconnection handshake and resets encoder ticks to `0` to synchronize with the new ROS 2 session.
* **`motor_command_callback`:** Receives `/cmd_vel` or `/cmd_vel_nav`, applies a clamp to ensure the minimum angular velocity is at least `MIN_ANGULAR_VELOCITY`, and calculates target wheel speeds using differential drive kinematics:
  $$v_{\text{left}} = v_x - \omega_z \cdot \left(\frac{\text{WHEEL\_BASE}}{2}\right)$$
  $$v_{\text{right}} = v_x + \omega_z \cdot \left(\frac{\text{WHEEL\_BASE}}{2}\right)$$
* **Watchdog:** If no command is received within `CMD_VEL_TIMEOUT_US` (150ms), the motors are automatically stopped for safety.

### B. Robot Specifications & Pinout: `main/robot_config.h`

#### GPIO Pin Connections
| Component | Function / Pin Name | ESP32-S3 GPIO |
| :--- | :--- | :--- |
| **Left Motor** | ENA (PWM Speed Control) | **GPIO 4** |
| | IN1 (Direction Control 1) | **GPIO 5** |
| | IN2 (Direction Control 2) | **GPIO 6** |
| **Right Motor** | ENB (PWM Speed Control) | **GPIO 8** |
| | IN3 (Direction Control 1) | **GPIO 10** |
| | IN4 (Direction Control 2) | **GPIO 9** |
| **Left Encoder** | Phase A / Phase B | **GPIO 7 / GPIO 15** |
| **Right Encoder**| Phase A / Phase B | **GPIO 11 / GPIO 16** |

#### Physical Specifications
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `PPR` | `325.0f` | Pulses Per Revolution (Encoder Ticks) |
| `WHEEL_DIAMETER` | `0.065f` | 65mm Wheel Diameter |
| `WHEEL_BASE` | `0.30f` | 300mm Track Width (distance between wheels) |
| `MAX_LINEAR_SPEED`| `1.30f` | Maximum calibrated speed (m/s) at 255 PWM |
| `MOTOR_LEFT_DEADBAND` | `90.0f` | Minimum PWM duty cycle to overcome static friction (Left) |
| `MOTOR_RIGHT_DEADBAND`| `90.0f` | Minimum PWM duty cycle to overcome static friction (Right) |
| `MIN_ANGULAR_VELOCITY`| `0.25f` | Minimum allowed angular velocity (rad/s) |

### C. Motor Control Hardware Driver: `main/motor_control.c`

* Uses the ESP32 **LEDC** peripheral at a **2kHz** frequency with an **8-bit duty cycle resolution** (0–255 range) on the `ENA` and `ENB` pins to control speed.
* `DieuKhienDongCo1` / `DieuKhienDongCo2` maps velocity commands and direction inputs to the raw driver GPIOs.

### D. Quadrature Encoder Driver: `main/encoder.c`

* Captures high-frequency encoder pulses via GPIO interrupts on both channels (A & B) in quadrature.
* Tracks directional tick increments and decrements.
* Provides a thread-safe function `get_pulses_atomically()` to fetch encoder ticks without race conditions.

### E. PID Controller: `main/pid_control.c`

* Controls the velocity of each wheel independently.
* **Feedforward:** Uses $ff = \left(\frac{\text{setpoint}}{\text{MAX\_LINEAR\_SPEED}}\right) \times 255.0f$ to instantly calculate the base voltage required for target speeds.
* **Feedback:** Adjusts error dynamically using $K_P = 100.0f$, $K_I = 80.0f$, and $K_D = 1.2f$.
* **Reversal Logic:** Resets the integral accumulator if the setpoint changes direction to prevent integral windup.
* **Deadband Compensation:** Rescales the output PWM using the hardware deadband value ($90.0f$) to overcome starting friction.

---

## 3. Important Integration Details for Radxa Host

> [!IMPORTANT]
> **Watchdog Safety Timeout**
> The ESP32 expects `/cmd_vel` or `/cmd_vel_nav` commands to be published regularly (at least every 150ms). If the command stream stops, the watchdog will immediately halt the robot.

> [!NOTE]
> **Auto-Reset on Host Restart**
> If you restart the Radxa agent or bringup stack, you do not need to manually reset the ESP32. The ESP32 will automatically detect the disconnect, reboot itself, reset its encoders to 0 to sync with the new odometry, and reconnect.

> [!WARNING]
> **Coordinate Framework Consistency**
> The wheel diameter ($65\text{mm}$) and wheel separation ($300\text{mm}$) defined in the ESP32 config must match the values used in `nav2_params.yaml` and the URDF configurations on the Radxa side.