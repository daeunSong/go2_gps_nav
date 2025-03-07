#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import math
import utm
import numpy as np
from transforms3d.euler import quat2euler


class OutdoorNavigationNode(Node):
    def __init__(self):
        super().__init__('outdoor_navigation_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)  # Linear speed in m/s
        self.declare_parameter('angular_speed', 0.5)  # Angular speed in rad/s
        self.declare_parameter('goal_threshold', 2.0)  # Distance threshold to consider goal reached in meters
        self.declare_parameter('yaw_threshold', 0.1)  # Yaw threshold in radians
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.yaw_threshold = self.get_parameter('yaw_threshold').value
        
        # State variables
        self.current_utm = None
        self.goal_utm = None
        self.yaw = None
        self.goal_reached = False
        
        # Subscribers
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/witmotion_imu/imu',
            self.imu_callback,
            10)
        
        self.goal_subscription = self.create_subscription(
            NavSatFix,
            '/goal',
            self.goal_callback,
            10)
        
        # Publisher
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Outdoor Navigation Node initialized')
    
    
    def calculate_utm_zone(self, latitude, longitude):
        """Calculate the UTM zone based on latitude and longitude."""
        # UTM zones are 6 degrees wide
        zone_number = int((longitude + 180) / 6) + 1
        
        # Special case for Norway and Svalbard
        if 56 <= latitude < 64 and 3 <= longitude < 12:
            zone_number = 32
            
        # Special cases for Svalbard
        if 72 <= latitude < 84:
            if 0 <= longitude < 9:
                zone_number = 31
            elif 9 <= longitude < 21:
                zone_number = 33
            elif 21 <= longitude < 33:
                zone_number = 35
            elif 33 <= longitude < 42:
                zone_number = 37
        
        # UTM zone letter
        if 84 >= latitude >= 72:
            zone_letter = 'X'
        elif 72 > latitude >= 64:
            zone_letter = 'W'
        elif 64 > latitude >= 56:
            zone_letter = 'V'
        elif 56 > latitude >= 48:
            zone_letter = 'U'
        elif 48 > latitude >= 40:
            zone_letter = 'T'
        elif 40 > latitude >= 32:
            zone_letter = 'S'
        elif 32 > latitude >= 24:
            zone_letter = 'R'
        elif 24 > latitude >= 16:
            zone_letter = 'Q'
        elif 16 > latitude >= 8:
            zone_letter = 'P'
        elif 8 > latitude >= 0:
            zone_letter = 'N'
        elif 0 > latitude >= -8:
            zone_letter = 'M'
        elif -8 > latitude >= -16:
            zone_letter = 'L'
        elif -16 > latitude >= -24:
            zone_letter = 'K'
        elif -24 > latitude >= -32:
            zone_letter = 'J'
        elif -32 > latitude >= -40:
            zone_letter = 'H'
        elif -40 > latitude >= -48:
            zone_letter = 'G'
        elif -48 > latitude >= -56:
            zone_letter = 'F'
        elif -56 > latitude >= -64:
            zone_letter = 'E'
        elif -64 > latitude >= -72:
            zone_letter = 'D'
        elif -72 > latitude >= -80:
            zone_letter = 'C'
        else:
            zone_letter = 'Z'  # Error flag
            
        return zone_number, zone_letter
        
    def gps_callback(self, msg):
        """Callback for the GPS data."""
        # Calculate UTM zone
        zone_number, zone_letter = self.calculate_utm_zone(msg.latitude, msg.longitude)
        
        # Convert GPS to UTM
        easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
        
        self.current_utm = (easting, northing)
        self.get_logger().info(f'Current UTM position: {self.current_utm}')
    
    def imu_callback(self, msg):
        """Callback for the IMU data."""
        # Extract orientation quaternion
        q = msg.orientation
        
        # Convert to Euler angles (roll, pitch, yaw)
        try:
            roll, pitch, self.yaw = quat2euler([q.w, q.x, q.y, q.z])
        except:
            self.get_logger().error(f'Wrong IMU oreintation value: {msg.orientation}')
        
        # IMU yaw is 0 when facing east and +90 degrees (pi/2 rad) when facing north
        # This already aligns with our ENU convention so no conversion needed
        self.get_logger().info(f'Current yaw: {self.yaw}')
    
    def goal_callback(self, msg):
        """Callback for the goal GPS position."""
        # Calculate UTM zone
        zone_number, zone_letter = self.calculate_utm_zone(msg.latitude, msg.longitude)
        
        # Convert goal GPS to UTM
        easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
        
        self.goal_utm = (easting, northing)
        self.goal_reached = False
        self.get_logger().info(f'New goal received: UTM {self.goal_utm}')
    
    def calculate_target_bearing(self):
        """Calculate the bearing from current position to goal in radians."""
        if self.current_utm is None or self.goal_utm is None:
            return None
        
        # Calculate difference in UTM coordinates
        dx = self.goal_utm[0] - self.current_utm[0]  # East
        dy = self.goal_utm[1] - self.current_utm[1]  # North
        
        # Calculate bearing using ENU convention
        # In ENU, 0 radians is east, pi/2 is north
        bearing = math.atan2(dy, dx)
        
        return bearing
    
    def calculate_distance_to_goal(self):
        """Calculate the Euclidean distance to the goal."""
        if self.current_utm is None or self.goal_utm is None:
            return float('inf')
        
        dx = self.goal_utm[0] - self.current_utm[0]
        dy = self.goal_utm[1] - self.current_utm[1]
        
        return math.sqrt(dx**2 + dy**2)
    
    def control_loop(self):
        """Main control loop to navigate to the goal."""
        if self.current_utm is None or self.yaw is None:
            self.get_logger().info('Waiting for GPS and IMU data...')
            return
        if self.goal_utm is None:
            self.get_logger().info('Waiting for target goal...')
            return
        
        if self.goal_reached:
            # Stop the robot if goal is reached
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)
            return
        
        # Calculate distance to goal
        distance = self.calculate_distance_to_goal()
        
        # Check if goal is reached
        if distance < self.goal_threshold:
            self.get_logger().info('Goal reached!')
            self.goal_reached = True
            # Stop the robot
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)
            return
        
        # Calculate target bearing
        target_bearing = self.calculate_target_bearing()
        if target_bearing is None:
            return
        
        # Calculate yaw error (difference between current yaw and target bearing)
        # Normalize to [-pi, pi]
        yaw_error = self.normalize_angle(target_bearing - self.yaw)
        
        # Create cmd_vel message
        cmd_vel = Twist()
        
        # Set angular velocity based on yaw error
        if abs(yaw_error) > self.yaw_threshold:
            # We need to turn first
            cmd_vel.angular.z = self.angular_speed * np.sign(yaw_error)
            cmd_vel.linear.x = 0.0
        else:
            # We're facing the right direction, move forward
            cmd_vel.linear.x = self.linear_speed
            # Small correction to maintain heading
            cmd_vel.angular.z = 0.5 * self.angular_speed * np.sign(yaw_error)
        
        # Publish command
        self.cmd_vel_publisher.publish(cmd_vel)
        
        self.get_logger().info(f'Distance to goal: {distance:.2f}m, Yaw error: {yaw_error:.2f}rad')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    
    node = OutdoorNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()