#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray
import actionlib

class Explore:
    def __init__(self):
        rospy.init_node('explore')
        
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.tf_listener = tf.TransformListener()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        self.map_data = None
        self.frontier = []
        self.blacklist = set()
        self.last_goal_time = rospy.Time.now()
        self.goal_timeout = rospy.Duration(30)  # 30 seconds timeout
        self.current_goal = None  # Store the current goal
        
        # 创建一个发布器来发布frontier标记
        self.frontier_pub = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=1)
        # 创建一个发布器来发布目标点标记
        self.target_pub = rospy.Publisher('/target_marker', Marker, queue_size=1)
        
        rospy.Timer(rospy.Duration(5), self.explore_timer_callback)

    def map_callback(self, msg):
        self.map_data = msg
        rospy.loginfo("Map updated. Size: %d x %d, Resolution: %.2f", 
                      msg.info.width, msg.info.height, msg.info.resolution)

    def get_frontiers(self):
        if self.map_data is None:
            return []

        frontiers = []
        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        
        for y in range(1, self.map_data.info.height - 1):
            for x in range(1, self.map_data.info.width - 1):
                if map_array[y, x] == 0:  # Free space
                    if -1 in map_array[y-1:y+2, x-1:x+2]:  # Check if unknown space is adjacent
                        wx = self.map_data.info.origin.position.x + x * self.map_data.info.resolution
                        wy = self.map_data.info.origin.position.y + y * self.map_data.info.resolution
                        frontiers.append((wx, wy))

        rospy.loginfo(f"Found {len(frontiers)} frontiers")
        return frontiers

    def explore_timer_callback(self, event):
        frontiers = self.get_frontiers()
        if not frontiers:
            rospy.loginfo("No frontiers found. Exploration complete.")
            return

        # 显示所有frontiers
        self.visualize_frontiers(frontiers)

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            rospy.logwarn("Failed to get robot pose. Skipping this cycle.")
            return

        # Filter out blacklisted frontiers
        valid_frontiers = [f for f in frontiers if f not in self.blacklist]

        if not valid_frontiers:
            rospy.loginfo("All frontiers are blacklisted. Clearing blacklist.")
            self.blacklist.clear()
            return

        # Choose frontier based on distance and a random factor
        chosen_frontier = min(valid_frontiers, 
                              key=lambda f: self.distance(robot_pose, f) + np.random.uniform(0, 0.5))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(chosen_frontier[0], chosen_frontier[1], 0)
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(goal, done_cb=self.goal_callback)
        self.last_goal_time = rospy.Time.now()
        self.current_goal = chosen_frontier  # Store the current goal
        self.visualize_target(chosen_frontier)  # Visualize the target
        rospy.loginfo(f"Sending goal: {chosen_frontier}")
        rospy.loginfo(f"Robot pose: ({robot_pose[0]:.2f}, {robot_pose[1]:.2f})")
        rospy.loginfo(f"Distance to goal: {self.distance(robot_pose, chosen_frontier):.2f}")

    def goal_callback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully")
        else:
            rospy.logwarn(f"Goal failed with status: {status}")
            # Add the failed frontier to the blacklist
            if self.current_goal:
                self.blacklist.add(self.current_goal)
                rospy.loginfo(f"Added frontier {self.current_goal} to blacklist")

        # Check for timeout
        if (rospy.Time.now() - self.last_goal_time) > self.goal_timeout:
            rospy.logwarn("Goal timeout reached. Selecting new frontier.")
            self.explore_timer_callback(None)

    def get_robot_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get robot pose: {e}")
            return None

    def distance(self, pose1, pose2):
        return np.sqrt((pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2)

    def visualize_frontiers(self, frontiers):
        marker_array = MarkerArray()
        
        for i, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = frontier[0]
            marker.pose.position.y = frontier[1]
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        self.frontier_pub.publish(marker_array)

    def visualize_target(self, target):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.target_pub.publish(marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    explorer = Explore()
    explorer.run()