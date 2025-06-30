#!/usr/bin/env python3
"""
 A* Planner 

"""
import csv
import math
import heapq

class AStarPlanner:
    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.rows = len(grid_map)
        self.cols = len(grid_map[0])

        # mapping for one-way directions
        self.DIRS = {'^': (-1, 0), '>': (0, 1), 'v': (1, 0), '<': (0, -1)}

    class Node:
        __slots__ = ('r', 'c', 'cost', 'parent')
        def __init__(self, r, c, cost, parent):
            self.r = r
            self.c = c
            self.cost = cost
            self.parent = parent

        def __repr__(self):
            return f"Node(r={self.r}, c={self.c}, cost={self.cost})"
        def __lt__(self, other):
            # tie-breaker for heap: compare cost
            return self.cost < other.cost

    def parse_cell(self, cell_str):
        """
        Parse a cell string "(up,right,down,left)"
        into a list of allowed one-way moves.
        """
        if not cell_str:
            return []
        parts = cell_str.strip().strip('()').split(',')
        moves = []
        # parts order corresponds to UP, RIGHT, DOWN, LEFT positions
        for sym in parts:
            if sym in self.DIRS:
                moves.append(self.DIRS[sym])
        return moves

    def get_neighbors(self, node):
        """
        Generate neighbor nodes based on one-way allowed directions.
        """
        neighbors = []
        for dr, dc in self.parse_cell(self.grid_map[node.r][node.c]):
            nr, nc = node.r + dr, node.c + dc
            # check bounds
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                cost = node.cost + math.hypot(dr, dc)
                neighbors.append(self.Node(nr, nc, cost, node))
        return neighbors

    @staticmethod
    def heuristic(a, b):
        # Manhattan distance for grid
        return abs(a.r - b.r) + abs(a.c - b.c)

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append((node.r, node.c))
            node = node.parent
        return list(reversed(path))

    def planning(self, start_r, start_c, goal_r, goal_c):
        start = self.Node(start_r, start_c, 0.0, None)
        goal = self.Node(goal_r, goal_c, 0.0, None)
        open_dict = {(start.r, start.c): start}
        closed = set()
        # heap holds tuples (f, Node); Node.__lt__ handles ties
        heap = [(self.heuristic(start, goal), start)]

        while heap:
            _, current = heapq.heappop(heap)
            key = (current.r, current.c)
            if key in closed:
                continue
            if current.r == goal.r and current.c == goal.c:
                return self.reconstruct_path(current)
            closed.add(key)

            for nbr in self.get_neighbors(current):
                nkey = (nbr.r, nbr.c)
                if nkey in closed:
                    continue
                if nkey not in open_dict or nbr.cost < open_dict[nkey].cost:
                    open_dict[nkey] = nbr
                    f = nbr.cost + self.heuristic(nbr, goal)
                    heapq.heappush(heap, (f, nbr))
        return None


def load_csv(path):
    with open(path, newline='') as f:
        return [row for row in csv.reader(f)]
    
if __name__ == '__main__':
    grid_raw = load_csv("/home/sai/projects/lexxpluss/map.csv")
    # convert grid_raw into grid_map[y][x]dr
    planner = AStarPlanner(grid_raw)
    print('start_goal:',grid_raw[0][1], grid_raw[0][7])
    path = planner.planning(0,1,0,7)
    print('Path:', path)
