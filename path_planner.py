#!/usr/bin/env python3

import numpy as np
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import tf_transformations

class PathPlanner:
    def __init__(self, node, params):
        self.node = node
        self.params = params
        
        # Create publisher
        self.path_pub = self.node.create_publisher(
            Path,
            self.params['topics']['path'],
            10)
    
    def plan_path(self, start_pos, goal_pos