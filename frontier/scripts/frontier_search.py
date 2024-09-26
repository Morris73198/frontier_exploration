#!/usr/bin/env python3

import numpy as np
from queue import Queue
from geometry_msgs.msg import Point

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

            point = Point()
            point.x = wx
            point.y = wy
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
        if self.costmap.data[idx] != -1:  # not unknown
            return False
        for nbr in self.nhood4(idx):
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

    def nhood4(self, index):
        w = self.costmap.info.width
        return [index - 1, index + 1, index - w, index + w]

    def nhood8(self, index):
        w = self.costmap.info.width
        return [index - 1, index + 1, index - w, index + w, 
                index - w - 1, index - w + 1, index + w - 1, index + w + 1]

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