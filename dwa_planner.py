#!/usr/bin/env python3

import numpy as np
import math
from scipy.spatial import KDTree

class DWAPlanner:
    def __init__(self, node, params):
        self.node = node
        
        # DWA parameters
        self.max_speed = params['dwa']['max_speed']
        self.min_speed = params['dwa']['min_speed']
        self.max_yawrate = params['dwa']['max_yawrate']
        self.max_accel = params['dwa']['max_accel']
        self.max_dyawrate = params['dwa']['max_dyawrate']
        self.v_resolution = params['dwa']['v_resolution']
        self.yawrate_resolution = params['dwa']['yawrate_resolution']
        self.dt = params['dwa']['dt']
        self.predict_time = params['dwa']['predict_time']
        self.to_goal_cost_gain = params['dwa']['to_goal_cost_gain']
        self.speed_cost_gain = params['dwa']['speed_cost_gain']
        self.obstacle_cost_gain = params['dwa']['obstacle_cost_gain']
        self.robot_radius = params['dwa']['robot_radius']
        self.obstacle_threshold = params['dwa']['obstacle_threshold']
        
        # Map parameters for obstacle detection
        self.map_resolution = params['map']['resolution']
        self.map_width = params['map']['width']
        self.map_height = params['map']['height']
        self.map_origin_x = params['map']['origin_x']
        self.map_origin_y = params['map']['origin_y']
        
    def compute_velocity(self, current_pos, heading, current_velocity, current_yawrate, 
                         target_pos, global_path, current_map):
        """Dynamic Window Approach for local planning"""
        if current_pos is None or global_path is None:
            return 0.0, 0.0  # No velocity or yaw rate
        
        # Dynamic window calculation
        Vs = [self.min_speed, self.max_speed,
             -self.max_yawrate, self.max_yawrate]
        
        Vd = [current_velocity - self.max_accel * self.dt,
              current_velocity + self.max_accel * self.dt,
              current_yawrate - self.max_dyawrate * self.dt,
              current_yawrate + self.max_dyawrate * self.dt]
        
        # Calculate dynamic window
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        
        # Find closest point on global path as local goal
        min_dist = float('inf')
        local_goal = None
        
        if len(global_path.poses) > 0:
            path_points = np.array([[pose.pose.position.x, pose.pose.position.y] 
                                    for pose in global_path.poses])
            
            # Use KDTree for efficient nearest point search
            tree = KDTree(path_points)
            dist, idx = tree.query(current_pos)
            
            # Look ahead a few points for better path following
            lookahead = min(idx + 5, len(path_points) - 1)
            local_goal = path_points[lookahead]
        else:
            # If no global path, use target directly
            local_goal = target_pos
        
        # Evaluate all possible velocity combinations
        best_v = 0.0
        best_w = 0.0
        best_score = -float('inf')
        
        # Discretize velocity space within dynamic window
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for w in np.arange(dw[2], dw[3], self.yawrate_resolution):
                # Predict trajectory for this control
                trajectory = self.predict_trajectory(current_pos, heading, v, w)
                
                # Calculate costs
                to_goal_cost = self.calc_to_goal_cost(trajectory, local_goal)
                speed_cost = self.calc_speed_cost(v)
                obstacle_cost = self.calc_obstacle_cost(trajectory, current_pos, current_map)
                
                # Total weighted cost - note we're maximizing
                final_cost = (self.to_goal_cost_gain * to_goal_cost +
                              self.speed_cost_gain * speed_cost +
                              self.obstacle_cost_gain * obstacle_cost)
                
                if final_cost > best_score:
                    best_v = v
                    best_w = w
                    best_score = final_cost
        
        return best_v, best_w
    
    def predict_trajectory(self, current_pos, heading, v, w):
        """Predict trajectory for given velocity and yaw rate"""
        trajectory = []
        time = 0.0
        x = current_pos[0]
        y = current_pos[1]
        theta = heading
        
        while time <= self.predict_time:
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            trajectory.append([x, y])
            time += self.dt
        
        return np.array(trajectory)
    
    def calc_to_goal_cost(self, trajectory, goal):
        """Calculate cost based on distance to goal"""
        # Use last point of trajectory
        dx = trajectory[-1, 0] - goal[0]
        dy = trajectory[-1, 1] - goal[1]
        dist = math.sqrt(dx**2 + dy**2)
        
        # Normalize cost (closer to goal is better)
        return 1.0 / (dist + 1e-6)
    
    def calc_speed_cost(self, v):
        """Calculate cost based on speed (prefer higher speeds)"""
        return v / self.max_speed
    
    def calc_obstacle_cost(self, trajectory, current_pos, current_map):
        """Calculate cost based on obstacle proximity"""
        if current_map is None:
            return 1.0  # No map, assume free
        
        min_dist = float('inf')
        
        # Transform trajectory points to map coordinates
        for point in trajectory:
            # Check if point is within map bounds
            map_x = int((point[0] - self.map_origin_x) / self.map_resolution)
            map_y = int((point[1] - self.map_origin_y) / self.map_resolution)
            
            if (0 <= map_x < self.map_width and
                0 <= map_y < self.map_height):
                
                # Check occupancy
                index = map_y * self.map_width + map_x
                if index < len(current_map.data):
                    occupancy = current_map.data[index]
                    
                    # If occupied, calculate distance to robot
                    if occupancy > self.obstacle_threshold:
                        dist = np.linalg.norm(point - current_pos) - self.robot_radius
                        min_dist = min(min_dist, dist)
        
        # If no obstacle found in trajectory
        if min_dist == float('inf'):
            return 1.0
        
        # Cost increases as distance decreases
        if min_dist <= 0.0:  # Collision
            return 0.0
        else:
            # Normalize cost (farther from obstacles is better)
            return min_dist / (self.predict_time * self.max_speed)