#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from queue import Queue
import actionlib
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray

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
        rospy.loginfo(f"Received costmap with size: {len(msg.data)}, width: {msg.info.width}, height: {msg.info.height}")

    def get_robot_pose(self):
        try:
            t = self.tf_listener.getLatestCommonTime(self.global_frame, self.robot_base_frame)
            position, quaternion = self.tf_listener.lookupTransform(self.global_frame, self.robot_base_frame, t)
            return Pose(position=Point(*position), orientation=quaternion)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get robot pose: {e}")
            return None

    def get_costmap(self):
        return self.costmap

class Frontier:
    def __init__(self):
        self.size = 0
        self.min_distance = float('inf')
        self.cost = 0
        self.initial = Point()
        self.centroid = Point()
        self.middle = Point()
        self.points = []

class FrontierSearch:
    def __init__(self, costmap, potential_scale, gain_scale, min_frontier_size):
        self.costmap = costmap
        self.potential_scale = potential_scale
        self.gain_scale = gain_scale
        self.min_frontier_size = min_frontier_size

    def search_from(self, position):
        mx, my = self.world_to_map(position.x, position.y)
        
        frontier_list = []
        map_size = self.costmap.info.width * self.costmap.info.height
        cell_states = ['unchecked'] * map_size

        bfs = Queue()
        clear = self.nearest_cell(mx, my, lambda c: c == 0)
        if clear is not None:
            bfs.put(clear)
        else:
            bfs.put(mx + my * self.costmap.info.width)
            rospy.logwarn("Could not find nearby clear cell to start search")

        while not bfs.empty():
            idx = bfs.get()
            
            if self.is_frontier_point(idx):
                new_frontier = self.build_frontier(idx, position, cell_states)
                if new_frontier.size * self.costmap.info.resolution >= self.min_frontier_size:
                    frontier_list.append(new_frontier)

            for nbr in self.nhood8(idx):
                if self.is_valid_index(nbr) and cell_states[nbr] == 'unchecked':
                    bfs.put(nbr)
                    cell_states[nbr] = 'queued'

            cell_states[idx] = 'processed'

        # Sort frontiers
        frontier_list.sort(key=lambda f: f.cost)

        rospy.loginfo(f"Found {len(frontier_list)} frontiers")
        return frontier_list

    def build_frontier(self, initial_cell, reference, cell_states):
        output = Frontier()
        output.centroid.x = 0
        output.centroid.y = 0
        output.size = 1
        output.min_distance = float('inf')

        ix, iy = self.index_to_cells(initial_cell)
        output.initial.x, output.initial.y = self.map_to_world(ix, iy)

        bfs = Queue()
        bfs.put(initial_cell)

        while not bfs.empty():
            idx = bfs.get()

            if cell_states[idx] == 'processed':
                continue

            mx, my = self.index_to_cells(idx)
            wx, wy = self.map_to_world(mx, my)

            point = Point(x=wx, y=wy, z=0)
            output.points.append(point)

            output.size += 1
            output.centroid.x += wx
            output.centroid.y += wy

            distance = self.distance(reference, point)
            if distance < output.min_distance:
                output.min_distance = distance
                output.middle.x = wx
                output.middle.y = wy

            for nbr in self.nhood8(idx):
                if self.is_valid_index(nbr) and cell_states[nbr] != 'processed' and self.is_frontier_point(nbr):
                    bfs.put(nbr)
                    cell_states[nbr] = 'queued'

            cell_states[idx] = 'processed'

        output.centroid.x /= output.size
        output.centroid.y /= output.size
        output.cost = self.frontier_cost(output)

        return output

    def frontier_cost(self, frontier):
        return (self.potential_scale * frontier.min_distance * self.costmap.info.resolution -
                self.gain_scale * frontier.size * self.costmap.info.resolution)

    def is_frontier_point(self, idx):
        if idx < 0 or idx >= len(self.costmap.data):
            rospy.logwarn(f"Invalid index {idx} for costmap data of length {len(self.costmap.data)}")
            return False
        if self.costmap.data[idx] != -1:  # not unknown
            return False
        for nbr in self.nhood4(idx):
            if nbr < 0 or nbr >= len(self.costmap.data):
                rospy.logwarn(f"Invalid neighbor index {nbr} for costmap data of length {len(self.costmap.data)}")
                continue
            if self.costmap.data[nbr] == 0:  # free
                return True
        return False

    def world_to_map(self, wx, wy):
        mx = int((wx - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
        my = int((wy - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        wx = mx * self.costmap.info.resolution + self.costmap.info.origin.position.x
        wy = my * self.costmap.info.resolution + self.costmap.info.origin.position.y
        return wx, wy

    def index_to_cells(self, index):
        return index % self.costmap.info.width, index // self.costmap.info.width

    def nhood4(self, idx):
        w = self.costmap.info.width
        h = self.costmap.info.height
        return [
            idx - 1 if idx % w > 0 else -1,
            idx + 1 if idx % w < w - 1 else -1,
            idx - w if idx >= w else -1,
            idx + w if idx < w * (h - 1) else -1
        ]

    def nhood8(self, idx):
        w = self.costmap.info.width
        h = self.costmap.info.height
        return [
            idx - 1 if idx % w > 0 else -1,
            idx + 1 if idx % w < w - 1 else -1,
            idx - w if idx >= w else -1,
            idx + w if idx < w * (h - 1) else -1,
            idx - w - 1 if idx % w > 0 and idx >= w else -1,
            idx - w + 1 if idx % w < w - 1 and idx >= w else -1,
            idx + w - 1 if idx % w > 0 and idx < w * (h - 1) else -1,
            idx + w + 1 if idx % w < w - 1 and idx < w * (h - 1) else -1
        ]

    def is_valid_index(self, index):
        return 0 <= index < len(self.costmap.data)

    def nearest_cell(self, mx, my, condition):
        index = mx + my * self.costmap.info.width
        if condition(self.costmap.data[index]):
            return index
        
        max_distance = max(self.costmap.info.width, self.costmap.info.height)
        for d in range(1, max_distance):
            for x in range(mx - d, mx + d + 1):
                for y in range(my - d, my + d + 1):
                    if 0 <= x < self.costmap.info.width and 0 <= y < self.costmap.info.height:
                        index = x + y * self.costmap.info.width
                        if condition(self.costmap.data[index]):
                            return index
        return None

    def distance(self, p1, p2):
        return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)



class Explore:
    def __init__(self):
        rospy.init_node('explore')
        
        self.costmap_client = Costmap2DClient()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        self.prev_distance = 0
        self.progress_timeout = rospy.Duration(rospy.get_param('~progress_timeout', 30.0))
        self.visualize = rospy.get_param('~visualize', False)
        self.planner_frequency = rospy.get_param('~planner_frequency', 1.0)
        self.potential_scale = rospy.get_param('~potential_scale', 1e-3)
        self.gain_scale = rospy.get_param('~gain_scale', 1.0)
        self.min_frontier_size = rospy.get_param('~minimum_frontier_size', 0.5)
        
        self.frontier_marker_pub = rospy.Publisher('frontier_markers', MarkerArray, queue_size=10)
        self.goal_marker_pub = rospy.Publisher('goal_marker', Marker, queue_size=10)
        self.prev_goal = Point()
        self.last_progress = rospy.Time.now()
        self.frontier_blacklist = []
        
        self.current_goal = None
        
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.planner_frequency), self.make_plan)

    def make_plan(self, event):
        pose = self.costmap_client.get_robot_pose()
        if pose is None:
            rospy.logwarn("Failed to get robot pose")
            return
        
        map = self.costmap_client.get_costmap()
        if map is None:
            rospy.logwarn("Failed to get costmap")
            return

        rospy.loginfo(f"Costmap size: {len(map.data)}, width: {map.info.width}, height: {map.info.height}")
        rospy.loginfo(f"Robot pose: x={pose.position.x}, y={pose.position.y}")

        frontier_search = FrontierSearch(map, self.potential_scale, self.gain_scale, self.min_frontier_size)
        frontiers = frontier_search.search_from(pose.position)

        if not frontiers:
            rospy.loginfo("No frontiers found, exploration complete")
            return

        if self.visualize:
            self.visualize_frontiers(frontiers)

        frontier = next((f for f in frontiers if not self.goal_on_blacklist(f.centroid)), None)
        if frontier is None:
            rospy.loginfo("All frontiers are blacklisted, exploration complete")
            return

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = map.header.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = frontier.centroid
        goal.target_pose.pose.orientation.w = 1.0
        
        self.current_goal = goal.target_pose.pose.position

        if self.prev_goal == goal.target_pose.pose.position:
            if frontier.min_distance < self.prev_distance:
                self.last_progress = rospy.Time.now()
        else:
            self.last_progress = rospy.Time.now()

        self.prev_goal = goal.target_pose.pose.position
        self.prev_distance = frontier.min_distance

        if (rospy.Time.now() - self.last_progress) > self.progress_timeout:
            rospy.loginfo("No progress for too long, blacklisting current goal")
            self.frontier_blacklist.append(self.prev_goal)
            return

        if self.visualize:
            self.visualize_goal()

        self.move_base_client.send_goal(goal, done_cb=self.goal_reached_cb)
        rospy.loginfo(f"Sending goal: {goal.target_pose.pose.position.x:.2f}, {goal.target_pose.pose.position.y:.2f}")

    def goal_reached_cb(self, status, result):
        if status == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("Goal aborted")
            self.frontier_blacklist.append(self.prev_goal)

        self.make_plan(None)  # Plan again

    def goal_on_blacklist(self, goal):
        for black_goal in self.frontier_blacklist:
            if self.distance(goal, black_goal) < 0.1:
                return True
        return False

    def visualize_frontiers(self, frontiers):
        f_markers = MarkerArray()
        
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.costmap_client.get_costmap().header.frame_id
            m.header.stamp = rospy.Time.now()
            m.ns = "frontiers"
            m.id = i
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0  # Blue color
            m.color.a = 1.0
            m.points = f.points
            f_markers.markers.append(m)
            
        self.frontier_marker_pub.publish(f_markers)

    def visualize_goal(self):
        if self.current_goal:
            goal_marker = Marker()
            goal_marker.header.frame_id = self.costmap_client.get_costmap().header.frame_id
            goal_marker.header.stamp = rospy.Time.now()
            goal_marker.ns = "current_goal"
            goal_marker.id = 0  # Single goal, so we can use a fixed ID
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position = self.current_goal
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.3
            goal_marker.scale.z = 0.3
            goal_marker.color.r = 1.0  # Red color
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            
            self.goal_marker_pub.publish(goal_marker)

    def distance(self, p1, p2):
        return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    explorer = Explore()
    explorer.run()