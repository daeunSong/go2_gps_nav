# GO2 GPS Navigation Package

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

