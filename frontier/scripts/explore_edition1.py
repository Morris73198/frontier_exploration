#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray
import actionlib
import numpy as np

class Explore:
    def __init__(self):
        rospy.init_node('explore')
        
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.tf_listener = tf.TransformListener()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        self.map_data = None
        self.frontier = []
        
        # 创建一个发布器来发布frontier标记和目标点标记
        self.frontier_pub = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=1)
        self.goal_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        
        rospy.Timer(rospy.Duration(5), self.explore_timer_callback)

    def map_callback(self, msg):
        self.map_data = msg

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

        return frontiers

    def explore_timer_callback(self, event):
        frontiers = self.get_frontiers()
        if not frontiers:
            rospy.loginfo("No frontiers found. Exploration complete.")
            return

        # 显示所有frontiers
        self.visualize_frontiers(frontiers)

        # 选择最近的frontier
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        nearest_frontier = min(frontiers, key=lambda f: self.distance(robot_pose, f))

        # 显示目标点为红色
        self.visualize_goal(nearest_frontier)

        # 发送目标到move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(nearest_frontier[0], nearest_frontier[1], 0)
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(goal)
        rospy.loginfo(f"Sending goal: {nearest_frontier}")

    def get_robot_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to get robot pose")
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

    def visualize_goal(self, goal):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal[0]
        marker.pose.position.y = goal[1]
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.goal_pub.publish(marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    explorer = Explore()
    explorer.run()