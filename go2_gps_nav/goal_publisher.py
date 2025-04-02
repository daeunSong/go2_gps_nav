#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import numpy as np
import time


class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')
        
        # Parameters
        self.declare_parameter('covariance_threshold', 200.0)  # Maximum acceptable covariance
        self.declare_parameter('collection_time', 10.0)  # Time to collect GPS readings in seconds
        self.declare_parameter('publish_rate', 1.0)  # Rate to publish goal in Hz
        
        self.covariance_threshold = self.get_parameter('covariance_threshold').value
        self.collection_time = self.get_parameter('collection_time').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # State variables
        self.gps_buffer = []
        self.is_collecting = False
        self.collection_start_time = None
        self.goal_calculated = False
        self.goal_msg = None
        
        # Subscribers and Publishers
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10)
        
        self.goal_publisher = self.create_publisher(
            NavSatFix,
            '/goal',
            10)
        
        # Timer for repeated goal publishing
        self.publish_timer = self.create_timer(1.0/self.publish_rate, self.publish_goal)
        
        self.get_logger().info('Goal Publisher Node initialized')
        self.get_logger().info(f'Waiting for GPS with covariance below {self.covariance_threshold}...')
    
    def get_max_covariance(self, msg):
        """Get the maximum value from the position covariance matrix."""
        if hasattr(msg, 'position_covariance'):
            # Return the maximum diagonal element (x, y, z variances)
            return max(msg.position_covariance[0], msg.position_covariance[4], msg.position_covariance[8])
        return float('inf')
    
    def gps_callback(self, msg):
        """Callback for the GPS data."""
        # Skip if we've already calculated the goal
        if self.goal_calculated:
            return
        
        # Check GPS covariance
        current_covariance = self.get_max_covariance(msg)
        
        if not self.is_collecting:
            if current_covariance < self.covariance_threshold:
                self.get_logger().info('GPS quality acceptable. Starting collection...')
                self.is_collecting = True
                self.collection_start_time = time.time()
                self.gps_buffer = []
            else:
                self.get_logger().info(f'Waiting for better GPS quality. Current covariance: {current_covariance:.2f}')
        
        # If we're in collection mode, store the GPS reading
        if self.is_collecting:
            self.gps_buffer.append((msg.latitude, msg.longitude, msg.altitude))
            
            # Check if we've collected for enough time
            elapsed_time = time.time() - self.collection_start_time
            remaining_time = max(0, self.collection_time - elapsed_time)
            
            if elapsed_time >= self.collection_time:
                self.calculate_goal()
            elif len(self.gps_buffer) % 5 == 0:  # Log progress every 5 readings
                self.get_logger().info(f'Collecting GPS readings: {len(self.gps_buffer)} readings, '
                                       f'{remaining_time:.1f} seconds remaining')
    
    def calculate_goal(self):
        """Calculate the average position and prepare it as the goal."""
        if len(self.gps_buffer) == 0:
            self.get_logger().error('No GPS readings collected, cannot calculate goal')
            self.is_collecting = False
            return
        
        # Calculate average position
        latitudes = [pos[0] for pos in self.gps_buffer]
        longitudes = [pos[1] for pos in self.gps_buffer]
        altitudes = [pos[2] for pos in self.gps_buffer]
        
        avg_latitude = sum(latitudes) / len(latitudes)
        avg_longitude = sum(longitudes) / len(longitudes)
        avg_altitude = sum(altitudes) / len(altitudes)
        
        # Calculate standard deviation to assess GPS quality
        std_lat = np.std(latitudes)
        std_lon = np.std(longitudes)
        
        # Create the goal message
        self.goal_msg = NavSatFix()
        self.goal_msg.header.frame_id = "gps"
        self.goal_msg.latitude = avg_latitude
        self.goal_msg.longitude = avg_longitude
        self.goal_msg.altitude = avg_altitude
        
        self.goal_calculated = True
        
        self.get_logger().info('Goal calculated successfully!')
        self.get_logger().info(f'Averaged position from {len(self.gps_buffer)} readings:')
        self.get_logger().info(f'Latitude: {avg_latitude}, Longitude: {avg_longitude}')
        self.get_logger().info(f'Position variation: σ_lat={std_lat:.8f}, σ_lon={std_lon:.8f}')
        self.get_logger().info(f'Will publish this goal continuously at {self.publish_rate} Hz')
        
    
    def publish_goal(self):
        """Publish the goal message at regular intervals."""
        if self.goal_calculated and self.goal_msg is not None:
            # Update the timestamp
            self.goal_msg.header.stamp = self.get_clock().now().to_msg()
            # Publish the goal
            self.goal_publisher.publish(self.goal_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = GoalPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()