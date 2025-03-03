#!/usr/bin/env python3

import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
import rclpy

class MapHandler:
    def __init__(self, node, params):
        self.node = node
        self.params = params
        
        # Map parameters
        self.map_resolution = params['map']['resolution']
        self.map_width = params['map']['width']
        self.map_height = params['map']['height']
        self.map_origin_x = params['map']['origin_x']
        self.map_origin_y = params['map']['origin_y']
        self.lidar_max_range = params['map']['lidar_max_range']
        
        # Initialize map data
        self.map_data = np.ones((self.map_height, self.map_width), dtype=np.int8) * -1  # -1 for unknown
        
        # Create publisher
        self.map_pub = self.node.create_publisher(
            OccupancyGrid,
            self.params['topics']['map'],
            10)
            
        # Initialize empty map
        self.initialize_map()
        
    def initialize_map(self):
        """Initialize empty occupancy grid map"""
        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"
        self.map.info.resolution = self.map_resolution
        self.map.info.width = self.map_width
        self.map.info.height = self.map_height
        self.map.info.origin.position.x = self.map_origin_x
        self.map.info.origin.position.y = self.map_origin_y
        self.map.info.origin.position.z = 0.0
        
        # Initialize with all unknown (-1)
        self.map.data = list(np.ones(self.map_width * self.map_height, dtype=np.int8) * -1)
        
        self.node.get_logger().info('Map initialized')
        
    def get_map(self):
        """Return the current map"""
        return self.map
        
    def publish_map(self, timestamp):
        """Publish the current occupancy grid map"""
        if self.map is None:
            return
        
        self.map.header.stamp = timestamp
        self.map_pub.publish(self.map)
        
    def update_map_from_lidar(self, scan_msg, robot_pos, robot_heading):
        """Process lidar data to update occupancy grid"""
        # Get robot position in map coordinates
        robot_x = robot_pos[0]
        robot_y = robot_pos[1]
        
        # Convert robot position to grid cells
        robot_cell_x = int((robot_x - self.map_origin_x) / self.map_resolution)
        robot_cell_y = int((robot_y - self.map_origin_y) / self.map_resolution)
        
        # Clear area around robot (local dynamic map)
        clear_radius = 10  # cells
        for i in range(-clear_radius, clear_radius + 1):
            for j in range(-clear_radius, clear_radius + 1):
                cell_x = robot_cell_x + i
                cell_y = robot_cell_y + j
                
                # Check if within map bounds
                if (0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height):
                    # Mark as free space (0)
                    index = cell_y * self.map_width + cell_x
                    self.map.data[index] = 0
        
        # Mark cells with obstacles
        angle = scan_msg.angle_min
        for range_reading in scan_msg.ranges:
            # Skip invalid readings
            if range_reading < scan_msg.range_min or range_reading > scan_msg.range_max or math.isnan(range_reading):
                angle += scan_msg.angle_increment
                continue
            
            # If reading is less than max range, we detected an obstacle
            if range_reading < self.lidar_max_range:
                # Calculate obstacle position in robot frame
                obstacle_x_robot = range_reading * math.cos(angle)
                obstacle_y_robot = range_reading * math.sin(angle)
                
                # Transform to map frame
                obstacle_x_map = robot_x + obstacle_x_robot * math.cos(robot_heading) - obstacle_y_robot * math.sin(robot_heading)
                obstacle_y_map = robot_y + obstacle_x_robot * math.sin(robot_heading) + obstacle_y_robot * math.cos(robot_heading)
                
                # Convert to grid cell
                obstacle_cell_x = int((obstacle_x_map - self.map_origin_x) / self.map_resolution)
                obstacle_cell_y = int((obstacle_y_map - self.map_origin_y) / self.map_resolution)
                
                # Check if within map bounds
                if (0 <= obstacle_cell_x < self.map_width and 0 <= obstacle_cell_y < self.map_height):
                    # Mark as occupied (100)
                    index = obstacle_cell_y * self.map_width + obstacle_cell_x
                    self.map.data[index] = 100
                    
                    # Update cells along the ray to be free
                    self.bresenham_line(robot_cell_x, robot_cell_y, obstacle_cell_x, obstacle_cell_y)
            
            angle += scan_msg.angle_increment
        
        self.node.get_logger().debug('Occupancy grid updated from lidar data')
        
    def bresenham_line(self, x0, y0, x1, y1):
        """Mark cells along line as free space using Bresenham's line algorithm"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            # Current cell is free if not the end cell (which is an obstacle)
            if (x0 != x1 or y0 != y1) and (0 <= x0 < self.map_width and 0 <= y0 < self.map_height):
                index = y0 * self.map_width + x0
                self.map.data[index] = 0  # free space
            
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy