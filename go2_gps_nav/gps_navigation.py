#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import numpy as np
import math
import pyproj

class GPSNavigationNode(Node):
    """
    Minimal GPS Navigation Node
    
    Subscribes to:
    - GPS location
    - Goal location
    
    Publishes:
    - Control commands (Twist)
    """
    
    def __init__(self):
        super().__init__('gps_navigation_node')
        
        # Declare parameters
        self.declare_parameter('utm_zone', 30)
        self.declare_parameter('goal_threshold', 2.0)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('control_frequency', 5.0)
        
        # Load parameters
        self.utm_zone = self.get_parameter('utm_zone').value
        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.max_speed = self.get_parameter('max_speed').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # UTM projection for GPS coordinates
        self.proj = pyproj.Proj(proj='utm', zone=self.utm_zone, ellps='WGS84')
        
        # State variables
        self.current_pos = None  # UTM coordinates [x, y]
        self.current_gps = None  # GPS coordinates
        self.target_gps = None   # Target GPS coordinates
        self.target_pos = None   # Target UTM coordinates
        self.heading = 0.0       # Current heading in radians
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10)
        
        self.goal_sub = self.create_subscription(
            NavSatFix,
            '/goal',
            self.goal_callback,
            10)
        
        # Control loop timer
        self.timer = self.create_timer(
            1.0 / self.control_frequency, 
            self.control_loop)
        
        self.get_logger().info('GPS Navigation Node initialized')
    
    def gps_callback(self, msg):
        """Process GPS location updates"""
        self.current_gps = msg
        x, y = self.proj(msg.longitude, msg.latitude)
        
        # Update position
        if self.current_pos is None:
            self.current_pos = np.array([x, y])
        else:
            # Calculate heading from position change
            new_pos = np.array([x, y])
            if np.linalg.norm(new_pos - self.current_pos) > 0.1:  # Only update if moved enough
                self.heading = math.atan2(new_pos[1] - self.current_pos[1], 
                                         new_pos[0] - self.current_pos[0])
            self.current_pos = new_pos
        
        self.get_logger().debug(f'Current position: {self.current_pos}, heading: {self.heading}')
    
    def goal_callback(self, msg):
        """Process navigation goal"""
        self.target_gps = msg
        x, y = self.proj(msg.longitude, msg.latitude)
        self.target_pos = np.array([x, y])
        
        self.get_logger().info(f'New goal received: {self.target_pos}')
    
    def control_loop(self):
        """Main navigation control loop"""
        if self.current_pos is None or self.target_pos is None:
            self.get_logger().info('Waiting for GPS and goal data...')
            return
        
        # Calculate distance to goal
        distance_to_goal = float(np.linalg.norm(self.current_pos - self.target_pos))
        
        # Check if goal reached
        if distance_to_goal < self.goal_threshold:
            self.get_logger().info('Goal reached!')
            # Stop the robot
            self.publish_cmd_vel(0.0, 0.0)
            return
        
        # Calculate direction to goal
        dx = self.target_pos[0] - self.current_pos[0]
        dy = self.target_pos[1] - self.current_pos[1]
        angle_to_goal = math.atan2(dy, dx)
        
        # Calculate heading error (difference between current heading and angle to goal)
        heading_error = self.normalize_angle(angle_to_goal - self.heading)
        
        # Calculate linear and angular velocity
        linear_vel = self.max_speed * (1 - abs(heading_error) / math.pi)
        angular_vel = 0.5 * heading_error  # Simple P controller
        
        # Ensure minimum linear velocity if not turning too much
        if abs(heading_error) < math.pi / 4:
            linear_vel = max(linear_vel, 0.1)
        else:
            linear_vel = 0.0  # Stop and turn if heading error is large
        
        # Publish velocity command
        self.publish_cmd_vel(linear_vel, angular_vel)
        
        # Log navigation status
        self.get_logger().debug(
            f'Distance to goal: {distance_to_goal:.2f}m, ' + 
            f'Heading error: {math.degrees(heading_error):.1f}°, ' +
            f'Cmd: linear={linear_vel:.2f}, angular={angular_vel:.2f}'
        )
    
    def publish_cmd_vel(self, linear_x, angular_z):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    node = GPSNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        node.publish_cmd_vel(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()