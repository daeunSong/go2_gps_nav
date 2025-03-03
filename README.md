# GO2 GPS Navigation Package

This package provides GPS-based navigation for robots using ROS2, featuring Dynamic Window Approach (DWA) for local obstacle avoidance.

## Testing

# GO2 GPS Navigation Testing Guide

This simple guide explains how to test the GPS navigation package by publishing to the `/fix` and `/goal` topics.


## Testing

1. Start the go2_bringup node:
```bash
ros2 launch go2_bringup go2.launch.py
```

2. Start the navigation node:

```bash
ros2 run go2_gps_nav gps_nav
```

3. Send GPS position data (continuously at 1 Hz):

```bash
ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{header: {frame_id: 'gps'}, latitude: 37.7749, longitude: -122.4194, altitude: 0.0}" -r 1
```

4. send a goal location:

```bash
ros2 topic pub -1 /goal sensor_msgs/msg/NavSatFix "{header: {frame_id: 'gps'}, latitude: 37.7751, longitude: -122.4190, altitude: 0.0}"
```

