# mecanum_hardware_interface

Custom [ros2_control](https://control.ros.org/) **system hardware interfaces** for the [RoboOmni](https://github.com/Armangb1/roboomni) mecanum-drive robot. Two transports are provided for talking to the ESP32 microcontroller — **micro-ROS over serial** (primary) and **Ethernet/UDP** (alternative).

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E)
![License](https://img.shields.io/badge/License-Apache_2.0-blue)

## Interfaces

| Plugin | Transport | Use case |
|---|---|---|
| `mecanum_hardware_interface/MicroRosInterface` | micro-ROS agent over serial | Primary — bridges ros2_control to the ESP32 via the [micro-ROS firmware](https://github.com/Armangb1/mecanum_microros_firmware) |
| `mecanum_hardware_interface/UdpHardware` | Raw UDP sockets | Alternative — pairs with the [UDP firmware](https://github.com/Armangb1/mecanum_udp_firmware) |

Both implement `hardware_interface::SystemInterface` and expose the same interfaces:

### Command interfaces

| Interface | Meaning |
|---|---|
| `<wheel_joint>/voltage` | Voltage command for each wheel motor (4 joints) |

### State interfaces

| Interface | Meaning |
|---|---|
| `<wheel_joint>/position` | Encoder position (4 joints) |
| `<wheel_joint>/velocity` | Wheel velocity (4 joints) |
| `<imu_sensor>/linear_acceleration.{x,y,z}` | Accelerometer |
| `<imu_sensor>/angular_velocity.{x,y,z}` | Gyroscope |

## How it works

**MicroRosInterface** bridges ros2_control to the micro-ROS topics published by the ESP32 firmware:

- `write()` publishes the 4 voltage commands on the `voltage_cmds` topic (`std_msgs/Float32MultiArray`)
- `read()` is driven by callbacks that consume `encoder` (positions + velocities) and `imu/data` topics
- Voltage commands are clamped to `±24 V` before transmission

**UdpHardware** talks to the microcontroller directly over UDP:

- Commands are sent as raw doubles to `192.168.1.3:8888`
- State is received on port `8889` (4 encoder doubles + 6 IMU doubles per frame)

## Integration

Declare it in the URDF via `ros2_control.xacro`:

```xml
<ros2_control name="MecanumMicroRos" type="system">
  <hardware>
    <plugin>mecanum_hardware_interface/MicroRosInterface</plugin>
  </hardware>
  <joint name="chassis_fr_wheel_joint">
    <command_interface name="voltage"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- fl, rl, rr joints ... -->
</ros2_control>
```

Controllers (e.g. [`mecanum_jacobian`](https://github.com/Armangb1/mecanum_jacobian_controller)) then claim the `voltage` command interfaces and the `position`/`velocity` state interfaces from this hardware.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mecanum_hardware_interface
source install/setup.bash
```

## Related repositories

- [`roboomni`](https://github.com/Armangb1/roboomni) — workspace, URDF, and system bringup
- [`mecanum_microros_firmware`](https://github.com/Armangb1/mecanum_microros_firmware) — ESP32 firmware this interface talks to
- [`mecanum_udp_firmware`](https://github.com/Armangb1/mecanum_udp_firmware) — UDP firmware variant

## License

Apache License 2.0. See [LICENSE](LICENSE).
