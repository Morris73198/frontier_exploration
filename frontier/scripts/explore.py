#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from costmap_client import Costmap2DClient
from frontier_search import FrontierSearch

class Explore:
    def __init__(self):
        self.costmap_client = Costmap2DClient()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        self.min_frontier_size = rospy.get_param('~minimum_frontier_size', 0.5)
        self.planner_frequency = rospy.get_param('~planner_frequency', 1.0)
        self.progress_timeout = rospy.Duration(rospy.get_param('~progress_timeout', 30.0))
        
        self.prev_distance = 0
        self.last_progress = rospy.Time.now()
        self.frontier_blacklist = []

        self.timer = rospy.Timer(rospy.Duration(1.0/self.planner_frequency), self.make_plan)

    def make_plan(self, event):
        pose = self.costmap_client.get_robot_pose()
        if pose is None:
            return

        frontiers = FrontierSearch(self.costmap_client.costmap, self.min_frontier_size).search_from(pose.position)
        
        if not frontiers:
            rospy.loginfo("No frontiers found, exploration completed.")
            return

        for frontier in frontiers:
            if not self.goal_on_blacklist(frontier.centroid):
                self.send_goal(frontier)
                break

    def send_goal(self, frontier):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.costmap_client.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = frontier.centroid[0]
        goal.target_pose.pose.position.y = frontier.centroid[1]
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(goal, done_cb=self.goal_reached_cb)

    def goal_reached_cb(self, status, result):
        if status == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("Goal aborted")
            # Add to blacklist
            # Implement blacklist logic here

        self.make_plan(None)  # Plan again

    def goal_on_blacklist(self, goal):
        # Implement blacklist checking logic
        return False

if __name__ == '__main__':
    rospy.init_node('explore')
    explorer = Explore()
    rospy.spin()