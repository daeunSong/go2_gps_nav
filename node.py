#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, LaserScan
from geometry_msgs.msg import Twist, Point
import numpy as np
import math
import tf2_ros
from tf2_ros import TransformException
import os

from gps_navigation.config import ConfigLoader
from gps_navigation.map_handler import MapHandler
from gps_navigation.dwa_planner import DWAPlanner
from gps_navigation.path_planner import PathPlanner
from gps_navigation.coordinate_utils import CoordinateTransformer
from gps_navigation.visualization import VisualizationPublisher

class GPSNavigationNode(Node):
    def __init__(self):
        super().__init__('gps_navigation_node')
        
        # Load parameters
        self.config_loader = ConfigLoader(self)
        self.params = self.config_loader.load_parameters()
        
        # Initialize components
        self.coord_transformer = CoordinateTransformer(self.params['utm']['zone'])
        self.map_handler = MapHandler(self, self.params)
        self.path_planner = PathPlanner(self, self.params)
        self.dwa_planner = DWAPlanner(self, self.params)
        self.visualization = VisualizationPublisher(self, self.params)
        
        # Current state
        self.current_pos = None  # UTM coordinates
        self.current_gps = None  # GPS coordinates
        self.target_gps = None   # Target GPS coordinates
        self.target_pos = None   # Target UTM coordinates
        self.heading = 0.0       # Current heading in radians
        self.current_velocity = 0.0  # Current forward velocity
        self.current_yawrate = 0.0   # Current yaw rate
        self.global_path = None  # Path from current position to goal
        
        # TF2 buffer for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.params['topics']['gps'],
            self.gps_callback,
            self.params['queue_sizes']['gps'])
        
        self.goal_sub = self.create_subscription(
            NavSatFix,
            self.params['topics']['goal'],
            self.goal_callback,
            10)
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.params['topics']['lidar'],
            self.lidar_callback,
            self.params['queue_sizes']['lidar'])
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.params['topics']['cmd_vel'],
            self.params['queue_sizes']['cmd_vel'])
        
        # Timers
        self.timer = self.create_timer(
            1.0 / self.params['frequencies']['control'], 
            self.control_loop)
        
        self.map_timer = self.create_timer(
            1.0 / self.params['frequencies']['map_publish'], 
            self.publish_map)
        
        self.tf_timer = self.create_timer(
            1.0 / self.params['frequencies']['tf_publish'], 
            self.broadcast_tf)
        
        self.get_logger().info('GPS Navigation Node has been initialized')
    
    def broadcast_tf(self):
        """Broadcast transform from map to base_link"""
        if self.current_pos is None:
            return
        
        self.visualization.broadcast_transform(
            self.current_pos, 
            self.heading, 
            self.get_clock().now().to_msg()
        )
    
    def publish_map(self):
        """Publish the current occupancy grid map"""
        self.map_handler.publish_map(self.get_clock().now().to_msg())
    
    def gps_callback(self, msg):
        """Handle GPS location updates"""
        self.current_gps = msg
        x, y = self.coord_transformer.gps_to_utm(msg.longitude, msg.latitude)
        
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
        
        self.get_logger().debug(f'Current position updated: {self.current_pos}, heading: {self.heading}')
    
    def goal_callback(self, msg):
        """Handle goal updates"""
        self.target_gps = msg
        x, y = self.coord_transformer.gps_to_utm(msg.longitude, msg.latitude)
        self.target_pos = np.array([x, y])
        
        # Plan global path when goal is received
        if self.current_pos is not None:
            self.global_path = self.path_planner.plan_path(
                self.current_pos, 
                self.target_pos, 
                self.get_clock().now().to_msg()
            )
            self.visualization.publish_goal_marker(
                self.target_pos, 
                self.get_clock().now().to_msg()
            )
        
        self.get_logger().info(f'New goal received: {self.target_pos}')
    
    def lidar_callback(self, msg):
        """Process lidar data to update occupancy grid"""
        if self.current_pos is None:
            return
        
        self.map_handler.update_map_from_lidar(
            msg, 
            self.current_pos, 
            self.heading
        )
    
    def control_loop(self):
        """Main control loop for navigation"""
        if self.current_pos is None or self.target_pos is None:
            self.get_logger().info('Waiting for GPS and goal data...')
            return
        
        # Check if we've reached the goal
        distance_to_goal = np.linalg.norm(self.current_pos - self.target_pos)
        if distance_to_goal < self.params['navigation']['goal_threshold']:
            self.get_logger().info('Goal reached!')
            # Stop the robot
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # Calculate control using DWA
        v, w = self.dwa_planner.compute_velocity(
            self.current_pos,
            self.heading,
            self.current_velocity,
            self.current_yawrate,
            self.target_pos,
            self.global_path,
            self.map_handler.get_map()
        )
        
        # Update current velocity and yaw rate
        self.current_velocity = v
        self.current_yawrate = w
        
        # Publish velocity command
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().debug(f'Cmd_vel published: linear={v}, angular={w}')


def main(args=None):
    rclpy.init(args=args)
    
    node = GPSNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()