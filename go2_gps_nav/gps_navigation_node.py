#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseStamped
import utm
import threading
import math
import geomag
import tf2_ros
import tf_transformations
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration


class GPS_localization(Node):
    def __init__(self):
        super().__init__('GPS_Localization_Node')
        
        self.br = tf2_ros.TransformBroadcaster(self)

        self._lock = threading.Lock()
        self.start_time = self.get_clock().now()
        self.calibrated = False
        self.cache = []  # list of (easting, northing)
        self.origin_easting = 0.0
        self.origin_northing = 0.0
        
        # Previous GPS fix & time (for heading logic)
        self.prev_easting = None
        self.prev_northing = None
        self.prev_fix_time = None

        # Previous relative pose & time (for velocity)
        self.prev_x_rel = 0.0
        self.prev_y_rel = 0.0
        self.prev_yaw = 0.0
        self.prev_pose_time = None

        # Filter state
        self.vel_alpha = 0.9          # higher => smoother
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_yaw_rate = 0.0

        # Direction (forward/back)
        self.direction = 1

        # Latest IMU orientation quaternion (x,y,z,w)
        self.latest_orientation = (0.0, 0.0, 0.0, 1.0)
        self.offset = 0.0
        
        # Subscribers
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            "fix", # atc edit
            self._gps_cb,
            10)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            "witmotion_imu/imu",
            self._imu_cb,
            100)
        
        self.goal_subscription = self.create_subscription(
            NavSatFix,
            'gps_goal',
            self._goal_cb,
            10)
        

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10)

        self.nav_goal_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10)
        
        self.get_logger().info('Outdoor Navigation Node initialized')
        
    def _gps_cb(self, msg):
        """Handle GPS fixes, calibration, pose + velocity publication."""
        # if msg.status.status < 0:
        #     self.get_logger().warn("Waiting for GPS good fix")
        #     return
        
        # Convert lat/lon -> UTM
        easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
        now = self.get_clock().now()

        with self._lock:
            # --- Calibration phase (first 5 s) ---
            if not self.calibrated:
                if (now - self.start_time) < Duration(seconds=5):
                    self.cache.append((easting, northing))
                    return
                xs, ys = zip(*self.cache)
                self.origin_easting = sum(xs) / len(xs)
                self.origin_northing = sum(ys) / len(ys)
                self.calibrated = True
                self.get_logger().info(f"Calibrated origin at ({self.origin_easting:.3f}, {self.origin_northing:.3f})")

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'utm'
                t.child_frame_id = 'map'

                t.transform.translation.x = self.origin_easting
                t.transform.translation.y = self.origin_northing
                t.transform.translation.z = 0.0

                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                # static transform: utm -> odom
                self.br.sendTransform(t)

                t.header.frame_id = 'map'
                t.child_frame_id = 'odom'
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                
                self.br.sendTransform(t)
                # init previous fix/time & pose/time
                self.prev_easting = easting
                self.prev_northing = northing
                self.prev_fix_time = now
                self.prev_pose_time = now
                return

            # --- After calibration: pose in odom frame --- 
            x_rel = easting - self.origin_easting
            y_rel = northing - self.origin_northing


            # extract IMU roll/pitch/yaw
            roll, pitch, imu_yaw = tf_transformations.euler_from_quaternion(self.latest_orientation)

            # decide heading from GPS vs IMU (same as before)
            heading_true = imu_yaw
            dt_fix = (now - self.prev_fix_time).nanoseconds / 1e9
            dx = easting - self.prev_easting
            dy = northing - self.prev_northing  
            dist = math.hypot(dx, dy)
            if dist > 1.0 and dt_fix < 8.0:
                heading_true = math.atan2(dy, dx)
                if self.direction < 0:
                    heading_true += math.pi
                heading_true = (heading_true + math.pi) % (2*math.pi) - math.pi
                # update offset
                self.offset = 0.9*self.offset + 0.1*(imu_yaw - heading_true)
                self.prev_easting, self.prev_northing, self.prev_fix_time = easting, northing, now

            # magnetic declination
            decl = self._get_declination(msg.latitude, msg.longitude, msg.altitude)
            yaw_corr = imu_yaw - self.offset + decl
            yaw_corr = (yaw_corr + math.pi) % (2*math.pi) - math.pi

            # build quaternion
            q_corr = tf_transformations.quaternion_from_euler(roll, pitch, yaw_corr)

            # --- Velocity estimation & filtering ---
            dt_pose = (now - self.prev_pose_time).nanoseconds / 1e9
            if dt_pose > 0:
                # raw linear velocities
                vx_raw = (x_rel - self.prev_x_rel) / dt_pose
                vy_raw = (y_rel - self.prev_y_rel) / dt_pose
                # raw angular velocity (yaw rate)
                dyaw = (yaw_corr - self.prev_yaw)
                # normalize angle difference
                dyaw = (dyaw + math.pi) % (2*math.pi) - math.pi
                yaw_rate_raw = dyaw / dt_pose

                # low-pass filter
                self.filtered_vx = (self.vel_alpha * self.filtered_vx +
                                    (1 - self.vel_alpha) * vx_raw)
                self.filtered_vy = (self.vel_alpha * self.filtered_vy +
                                    (1 - self.vel_alpha) * vy_raw)
                self.filtered_yaw_rate = (self.vel_alpha * self.filtered_yaw_rate +
                                          (1 - self.vel_alpha) * yaw_rate_raw)

            # update previous pose/time
            self.prev_x_rel, self.prev_y_rel = x_rel, y_rel
            self.prev_yaw = yaw_corr
            self.prev_pose_time = now

            # --- Broadcast and publish ---
            # tf: odom -> base_link
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = x_rel
            t.transform.translation.y = y_rel
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q_corr[1]
            t.transform.rotation.y = q_corr[2]
            t.transform.rotation.z = q_corr[3]
            t.transform.rotation.w = q_corr[0]

            self.br.sendTransform(t)

            t.header.frame_id = 'utm'
            t.child_frame_id = 'map'
            t.transform.translation.x = self.origin_easting
            t.transform.translation.y = self.origin_northing
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.br.sendTransform(t)

            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            self.br.sendTransform(t)

            # Odometry msg
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            # pose
            odom.pose.pose.position.x = x_rel
            odom.pose.pose.position.y = y_rel
            odom.pose.pose.orientation.x = q_corr[1]
            odom.pose.pose.orientation.y = q_corr[2]
            odom.pose.pose.orientation.z = q_corr[3]
            odom.pose.pose.orientation.w = q_corr[0]

            # twist
            odom.twist.twist.linear.x = self.filtered_vx
            odom.twist.twist.linear.y = self.filtered_vy
            odom.twist.twist.angular.z = self.filtered_yaw_rate

            self.odom_pub.publish(odom)

    def _get_declination(self, lat, lon, alt):
        """Return magnetic declination (radians) at given lat, lon, alt."""
        deg = geomag.declination(lat, lon, alt)
        self.get_logger().info(f"Magnetic declination: {deg:.3f}°")
        return -math.radians(deg)
    
    def _imu_cb(self, msg):
        """Cache the latest IMU orientation."""
        q = msg.orientation
        with self._lock:
            self.latest_orientation = (q.w, q.x, q.y, q.z)     
    
    def _goal_cb(self, msg):
        """Handle GPS goal messages."""
        if not self.calibrated:
            self.get_logger().warn("Position not initialized. Skipping")
            return
        
        # Convert lat/lon -> UTM
        easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
        now = self.get_clock().now()

        goal = PoseStamped()
        goal.header.stamp = now
        goal.header.frame_id = 'odom'
        goal.pose.position.x = easting - self.origin_easting
        goal.pose.position.y = northing - self.origin_northing
        goal.pose.orientation.w = 1.0

        self.get_logger().info(f'New goal received: UTM {goal}')
        self.nav_goal_publisher.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    
    node = GPS_localization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
