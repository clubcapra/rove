#!/usr/bin/env python3

"""
Test script for running a simple behavior tree for autonomous navigation.
Taken from https://github.com/sea-bass/turtlebot3_behavior_demos/blob/main/tb3_autonomy/scripts/autonomy_node.py


Example usage:
  ros2 run rove_behavior autonomy_node
"""

import os
import yaml
import random
import rclpy
from rclpy.node import Node
import time
import py_trees
import py_trees_ros
from py_trees.common import OneShotPolicy
from ament_index_python.packages import get_package_share_directory

from rove_navigation.behavior.navigation import GoToPose, GetLocationFromQueue


class AutonomyBehavior(Node):
    def __init__(self):
        super().__init__("autonomy_node")
        self.get_logger().info("AutonomyBehavior node has been initialized.")

        # Defines object locations as [x, y, theta]
        self.locations = {
            "location1": [0.0, 0.0, -1.571],
            "location2": [2.0, 0.0, 1.571],
            "location3": [2.0, 2.0, 0.0],
            "location4": [0.0, 2.0, 3.142],
        }

        self.loc_list = list(self.locations.keys())

        # Create and setup the behavior tree
        self.tree = self.create_naive_tree()

    
    def create_naive_tree(self):
        """Create behavior tree with explicit nodes for each location."""
        seq = py_trees.composites.Sequence(name="navigation", memory=True)
        root = py_trees.decorators.OneShot(
            name="root", child=seq, policy=OneShotPolicy.ON_SUCCESSFUL_COMPLETION
        )
        tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=False)
        tree.setup(timeout=15.0, node=self)

        for loc in self.loc_list:
            pose = self.locations[loc]
            seq.add_child(GoToPose(f"go_to_{loc}", pose, self))
            self.get_logger().info(f"Added GoToPose node for {loc} with pose {pose}")

        return tree

    def execute(self, period=0.5):
        """Executes the behavior tree at the specified period."""
        self.tree.tick_tock(period_ms=period * 1000.0)
        rclpy.spin(self.tree.node)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    behavior = AutonomyBehavior()
    behavior.execute()
    rclpy.shutdown()

if __name__ == "__main__":
    main()