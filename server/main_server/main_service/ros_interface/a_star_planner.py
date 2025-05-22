#!/usr/bin/env python3
import heapq
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped

class AStarPlanner:
    def __init__(self, grid: OccupancyGrid):
        self.grid   = grid
        self.width  = grid.info.width
        self.height = grid.info.height
        self.res    = grid.info.resolution
        self.origin = grid.info.origin.position
        # YAML 설정 반영
        self.free_thresh = 25   # free_thresh:0.25→25
        self.occ_thresh  = 65   # occupied_thresh:0.65→65

    def heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def world_to_map(self, x, y):
        mx = int((x - self.origin.x) / self.res)
        my = int((y - self.origin.y) / self.res)
        return mx, my

    def map_to_world(self, mx, my):
        x = mx * self.res + self.origin.x + self.res/2
        y = my * self.res + self.origin.y + self.res/2
        return x, y

    def is_free(self, mx, my):
        if 0 <= mx < self.width and 0 <= my < self.height:
            val = self.grid.data[my * self.width + mx]
            return 0 <= val <= self.free_thresh
        return False

    def compute_path(self, start: Pose, goal: Pose):
        start_map = self.world_to_map(start.position.x, start.position.y)
        goal_map  = self.world_to_map(goal.position.x,   goal.position.y)
        open_set  = [(0, start_map)]
        came_from = {}
        gscore    = {start_map: 0}
        dirs      = [(1,0),(-1,0),(0,1),(0,-1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_map:
                break
            for d in dirs:
                nbr = (current[0]+d[0], current[1]+d[1])
                if not self.is_free(*nbr):
                    continue
                tentative = gscore[current] + 1
                if tentative < gscore.get(nbr, float('inf')):
                    came_from[nbr] = current
                    gscore[nbr]   = tentative
                    f = tentative + self.heuristic(nbr, goal_map)
                    heapq.heappush(open_set, (f, nbr))

        path = []
        node = goal_map
        while node != start_map:
            wm = self.map_to_world(*node)
            ps = PoseStamped()
            ps.header.frame_id = self.grid.header.frame_id
            ps.pose.position.x = wm[0]
            ps.pose.position.y = wm[1]
            ps.pose.orientation.w = 1.0
            path.append(ps)
            node = came_from.get(node)
            if node is None:
                return []
        path.reverse()
        return path
