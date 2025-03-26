# GPS + IMU Navigation to the Goal

## Package Structure

```
my_ros2_ws/src/go2_gps_nav/
├── go2_gps_nav/
│   ├── __init__.py
│   └── gps_navigation_node.py   # Main navigation code
├── config/
│   └── navigation_params.yaml   # Configuration parameters
├── launch/
│   └── gps_navigation.launch.py # Launch file
├── resource/
│   └── go2_gps_nav              # Marker file for package discovery
├── package.xml
├── setup.py
└── setup.cfg
```

## Dependencies

- ROS2 Foxy
- Python 3
- tf_transformations
- numpy


## Usage

1. Launch the navigation node:
   ```bash
   ros2 launch go2_gps_nav gps_navigation.launch.py
   ```

2. Send a GPS goal:
   ```bash
   ros2 topic pub -1 /goal sensor_msgs/msg/NavSatFix "{latitude: 38.8277645, longitude: -77.3051732999}"
   ```

## Topic Interface

### Subscribed Topics
- `/fix` ([sensor_msgs/NavSatFix](https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html)) - Current GPS position
- `/imu` ([sensor_msgs/Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html)) - IMU data for heading
- `/goal` ([sensor_msgs/NavSatFix](https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html)) - Target GPS position

### Published Topics
- `/cmd_vel` ([geometry_msgs/Twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)) - Robot velocity commands

## Configuration

Edit the configuration file at `config/navigation_params.yaml` to adjust navigation parameters:

```yaml
gps_navigation_node:
  ros__parameters:

    linear_speed: 0.5      # linear speed (m/s)
    angular_speed: 0.5     # angular speed (rad/s)
    goal_threshold: 2.0    # Distance tolerance for goal (meters)
    yaw_threshold: 0.1     # Heading tolerance (radians)
```