#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import math
import utm
import numpy as np
from transforms3d.euler import quat2euler
from collections import deque


class OutdoorNavigationNode(Node):
    def __init__(self):
        super().__init__('outdoor_navigation_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)  # Linear speed in m/s
        self.declare_parameter('angular_speed', 0.5)  # Angular speed in rad/s
        self.declare_parameter('goal_threshold', 2.0)  # Distance threshold to consider goal reached in meters
        self.declare_parameter('yaw_threshold', 0.1)  # Yaw threshold in radians
        self.declare_parameter('gps_filter_window', 10)  # Number of GPS readings to average
        self.declare_parameter('init_duration', 10.0)  # Initialization duration in seconds
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.yaw_threshold = self.get_parameter('yaw_threshold').value
        self.gps_filter_window = self.get_parameter('gps_filter_window').value
        self.init_duration = self.get_parameter('init_duration').value
        
        # State variables
        self.current_utm = None
        self.filtered_utm = None
        self.goal_utm = None
        self.yaw = None
        self.goal_reached = False
        
        # GPS filtering buffer
        self.gps_buffer = deque(maxlen=self.gps_filter_window)
        
        # Initialization state
        self.is_initialized = False
        self.init_start_time = self.get_clock().now()
        self.init_gps_buffer = []
        
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

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Outdoor Navigation Node initialized')
        self.get_logger().info(f'Starting initialization phase for {self.init_duration} seconds...')
    
    def gps_callback(self, msg):
        """Callback for the GPS data."""
        # Convert GPS to UTM
        try:
            easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
            
            # Store raw UTM position
            self.current_utm = (easting, northing)
            
            # During initialization phase, collect GPS readings
            if not self.is_initialized:
                self.init_gps_buffer.append(self.current_utm)
                return
            
            # Add to filter buffer for normal operation
            self.gps_buffer.append(self.current_utm)
            
            # Apply filtering (moving average)
            if len(self.gps_buffer) > 0:
                avg_easting = sum(pos[0] for pos in self.gps_buffer) / len(self.gps_buffer)
                avg_northing = sum(pos[1] for pos in self.gps_buffer) / len(self.gps_buffer)
                self.filtered_utm = (avg_easting, avg_northing)
        
        except Exception as e:
            self.get_logger().error(f'Error processing GPS data: {e}')
    
    def imu_callback(self, msg):
        """Callback for the IMU data."""
        # Extract orientation quaternion
        q = msg.orientation
        
        try:
            # Convert to Euler angles (roll, pitch, yaw)
            roll, pitch, self.yaw = quat2euler([q.w, q.x, q.y, q.z])
        except Exception as e:
            self.get_logger().error(f'Wrong IMU orientation value: {msg.orientation}')
    
    def goal_callback(self, msg):
        """Callback for the goal GPS position."""
        if not self.is_initialized:
            self.get_logger().warn('Cannot set goal: System not initialized')
            return
        
        try:
            # Convert goal GPS to UTM
            easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
            
            self.goal_utm = (easting, northing)
            self.goal_reached = False
            
            # Calculate distance to goal
            if self.filtered_utm is not None:
                dx = self.goal_utm[0] - self.filtered_utm[0]
                dy = self.goal_utm[1] - self.filtered_utm[1]
                distance = math.sqrt(dx**2 + dy**2)
                self.get_logger().info(f'New goal received: UTM {self.goal_utm}, '
                                      f'Distance: {distance:.2f}m')
            else:
                self.get_logger().info(f'New goal received: UTM {self.goal_utm}')
        
        except Exception as e:
            self.get_logger().error(f'Error processing goal: {e}')
    
    def calculate_target_bearing(self):
        """Calculate the bearing from current position to goal in radians."""
        if self.filtered_utm is None or self.goal_utm is None:
            return None
        
        # Calculate difference in UTM coordinates
        dx = self.goal_utm[0] - self.filtered_utm[0]  # East
        dy = self.goal_utm[1] - self.filtered_utm[1]  # North
        
        # Calculate bearing using ENU convention
        # In ENU, 0 radians is east, pi/2 is north
        bearing = math.atan2(dy, dx)
        
        return bearing
    
    def calculate_distance_to_goal(self):
        """Calculate the Euclidean distance to the goal."""
        if self.filtered_utm is None or self.goal_utm is None:
            return float('inf')
        
        dx = self.goal_utm[0] - self.filtered_utm[0]
        dy = self.goal_utm[1] - self.filtered_utm[1]
        
        return math.sqrt(dx**2 + dy**2)
    
    def control_loop(self):
        """Main control loop to navigate to the goal."""
        # Check if initialization is complete
        if not self.is_initialized:
            elapsed = (self.get_clock().now() - self.init_start_time).nanoseconds / 1e9
            
            if elapsed >= self.init_duration and len(self.init_gps_buffer) > 0:
                # Calculate average UTM coordinates
                easting_sum = sum(utm_pos[0] for utm_pos in self.init_gps_buffer)
                northing_sum = sum(utm_pos[1] for utm_pos in self.init_gps_buffer)
                
                avg_easting = easting_sum / len(self.init_gps_buffer)
                avg_northing = northing_sum / len(self.init_gps_buffer)
                
                # Initialize the filtered_utm with the average
                self.filtered_utm = (avg_easting, avg_northing)
                
                # Mark as initialized
                self.is_initialized = True
                self.get_logger().info('Initialization complete!')
                self.get_logger().info(f'Initial position: UTM {self.filtered_utm}')
            else:
                # Stop the robot during initialization
                cmd_vel = Twist()
                self.cmd_vel_publisher.publish(cmd_vel)
                return
        
        # Wait for required data
        if self.filtered_utm is None or self.yaw is None:
            self.get_logger().info('Waiting for filtered GPS and IMU data...')
            # Stop the robot
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)
            return
        
        if self.goal_utm is None:
            self.get_logger().info('Waiting for target goal...')
            # Stop the robot
            cmd_vel = Twist()
            self.cmd_vel_publisher.publish(cmd_vel)
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