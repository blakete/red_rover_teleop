# Red Rover Teleop

Xbox 360 controller teleoperation for the Red Rover platform.

Provides a ROS 2 node that reads Xbox 360 controller input via `joy_node` and publishes `geometry_msgs/Twist` velocity commands and `std_msgs/Bool` arm commands for differential drive control.

## Control Scheme

| Input | Action |
|---|---|
| Left Stick Y | Forward / Backward (linear velocity) |
| Left Stick X | Turn left / right (angular velocity) |
| A button | Arm motors |
| B button | Disarm motors (emergency stop) |

## Safety

- Starts **disarmed** -- press A to enable movement
- Full stick deflection = max speed (1.0 m/s linear, 1.5 rad/s angular)
- Immediate zero on stick release
- Zero velocity + disarm sent on shutdown

## Files

```
red_rover_teleop/
├── docker/
│   └── Dockerfile              # Container image (ros:foxy + joy)
├── src/
│   ├── xbox_teleop_node.py     # ROS 2 teleop node
│   └── teleop_control_xbox360.py  # Standalone teleop (no ROS)
├── launch/
│   └── teleop.launch.py        # ROS 2 launch file
├── .gitignore
├── LICENSE
└── README.md
```

## ROS 2 Topics

### Published
- `cmd_vel` (`geometry_msgs/Twist`) -- velocity commands
- `arm` (`std_msgs/Bool`) -- motor arm/disarm

### Subscribed
- `joy` (`sensor_msgs/Joy`) -- Xbox controller input from `joy_node`

## Docker Usage

This container is typically run via the orchestrator's `docker-compose.yml`. The DDS discovery client config is volume-mounted from the bridge repo at runtime.

```bash
# From the orchestrator root
docker compose up teleop
```

## Standalone Usage (no ROS)

The standalone script reads the controller directly without ROS:

```bash
python src/teleop_control_xbox360.py
python src/teleop_control_xbox360.py --serial /dev/ttyUSB0
```
