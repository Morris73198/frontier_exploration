#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

class Costmap2DClient:
    def __init__(self):
        self.costmap = None
        self.tf_listener = tf.TransformListener()
        self.robot_base_frame = rospy.get_param('~robot_base_frame', 'base_link')
        self.global_frame = None
        self.costmap_sub = rospy.Subscriber('costmap', OccupancyGrid, self.costmap_callback)

    def costmap_callback(self, msg):
        self.costmap = msg
        self.global_frame = msg.header.frame_id

    def get_robot_pose(self):
        try:
            t = self.tf_listener.getLatestCommonTime(self.global_frame, self.robot_base_frame)
            position, quaternion = self.tf_listener.lookupTransform(self.global_frame, self.robot_base_frame, t)
            return Pose(position=position, orientation=quaternion)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to get robot pose")
            return None

    def get_costmap(self):
        return self.costmap