#!/usr/bin/env python3

import os
import math
import yaml
from ament_index_python.packages import get_package_share_directory

class ConfigLoader:
    def __init__(self, node):
        self.node = node
        self.declare_parameters()
        
    def declare_parameters(self):
        """Declare all parameters with default values"""
        # ROS topics and general parameters
        self.node.declare_parameter('topics.gps', '/fix')
        self.node.declare_parameter('topics.goal', '/goal')
        self.node.declare_parameter('topics.lidar', '/scan')
        self.node.declare_parameter('topics.cmd_vel', '/cmd_vel')
        self.node.declare_parameter('topics.path', '/global_path')
        self.node.declare_parameter('topics.marker', '/goal_marker')
        self.node.declare_parameter('topics.map', '/map')
        
        self.node.declare_parameter('queue_sizes.gps', 10)
        self.node.declare_parameter('queue_sizes.lidar', 10)
        self.node.declare_parameter('queue_sizes.cmd_vel', 10)
        
        self.node.declare_parameter('frequencies.control', 5.0)
        self.node.declare_parameter('frequencies.map_publish', 1.0)
        self.node.declare_parameter('frequencies.tf_publish', 10.0)
        
        # UTM parameters
        self.node.declare_parameter('utm.zone', 30)
        
        # DWA parameters
        self.node.declare_parameter('dwa.max_speed', 0.5)
        self.node.declare_parameter('dwa.min_speed', 0.0)
        self.node.declare_parameter('dwa.max_yawrate', 40.0)
        self.node.declare_parameter('dwa.max_accel', 0.2)
        self.node.declare_parameter('dwa.max_dyawrate', 40.0)
        self.node.declare_parameter('dwa.v_resolution', 0.01)
        self.node.declare_parameter('dwa.yawrate_resolution', 0.1)
        self.node.declare_parameter('dwa.dt', 0.1)
        self.node.declare_parameter('dwa.predict_time', 3.0)
        self.node.declare_parameter('dwa.to_goal_cost_gain', 1.0)
        self.node.declare_parameter('dwa.speed_cost_gain', 1.0)
        self.node.declare_parameter('dwa.obstacle_cost_gain', 1.0)
        self.node.declare_parameter('dwa.robot_radius', 0.5)
        self.node.declare_parameter('dwa.obstacle_threshold', 50)
        
        # Map parameters
        self.node.declare_parameter('map.resolution', 0.1)
        self.node.declare_parameter('map.width', 1000)
        self.node.declare_parameter('map.height', 1000)
        self.node.declare_parameter('map.origin_x', -50.0)
        self.node.declare_parameter('map.origin_y', -50.0)
        self.node.declare_parameter('map.lidar_max_range', 30.0)
        
        # Navigation parameters
        self.node.declare_parameter('navigation.goal_threshold', 1.0)
        
    def load_parameters(self):
        """Load parameters from declared ROS parameters into a dictionary"""
        params = {}
        
        # Topics and general parameters
        params['topics'] = {
            'gps': self.node.get_parameter('topics.gps').value,
            'goal': self.node.get_parameter('topics.goal').value,
            'lidar': self.node.get_parameter('topics.lidar').value,
            'cmd_vel': self.node.get_parameter('topics.cmd_vel').value,
            'path': self.node.get_parameter('topics.path').value,
            'marker': self.node.get_parameter('topics.marker').value,
            'map': self.node.get_parameter('topics.map').value
        }
        
        params['queue_sizes'] = {
            'gps': self.node.get_parameter('queue_sizes.gps').value,
            'lidar': self.node.get_parameter('queue_sizes.lidar').value,
            'cmd_vel': self.node.get_parameter('queue_sizes.cmd_vel').value
        }
        
        params['frequencies'] = {
            'control': self.node.get_parameter('frequencies.control').value,
            'map_publish': self.node.get_parameter('frequencies.map_publish').value,
            'tf_publish': self.node.get_parameter('frequencies.tf_publish').value
        }
        
        # UTM parameters
        params['utm'] = {
            'zone': self.node.get_parameter('utm.zone').value
        }
        
        # DWA parameters
        params['dwa'] = {
            'max_speed': self.node.get_parameter('dwa.max_speed').value,
            'min_speed': self.node.get_parameter('dwa.min_speed').value,
            'max_yawrate': self.node.get_parameter('dwa.max_yawrate').value * math.pi / 180.0,
            'max_accel': self.node.get_parameter('dwa.max_accel').value,
            'max_dyawrate': self.node.get_parameter('dwa.max_dyawrate').value * math.pi / 180.0,
            'v_resolution': self.node.get_parameter('dwa.v_resolution').value,
            'yawrate_resolution': self.node.get_parameter('dwa.yawrate_resolution').value * math.pi / 180.0,
            'dt': self.node.get_parameter('dwa.dt').value,
            'predict_time': self.node.get_parameter('dwa.predict_time').value,
            'to_goal_cost_gain': self.node.get_parameter('dwa.to_goal_cost_gain').value,
            'speed_cost_gain': self.node.get_parameter('dwa.speed_cost_gain').value,
            'obstacle_cost_gain': self.node.get_parameter('dwa.obstacle_cost_gain').value,
            'robot_radius': self.node.get_parameter('dwa.robot_radius').value,
            'obstacle_threshold': self.node.get_parameter('dwa.obstacle_threshold').value
        }
        
        # Map parameters
        params['map'] = {
            'resolution': self.node.get_parameter('map.resolution').value,
            'width': self.node.get_parameter('map.width').value,
            'height': self.node.get_parameter('map.height').value,
            'origin_x': self.node.get_parameter('map.origin_x').value,
            'origin_y': self.node.get_parameter('map.origin_y').value,
            'lidar_max_range': self.node.get_parameter('map.lidar_max_range').value
        }
        
        # Navigation parameters
        params['navigation'] = {
            'goal_threshold': self.node.get_parameter('navigation.goal_threshold').value
        }
        
        return params