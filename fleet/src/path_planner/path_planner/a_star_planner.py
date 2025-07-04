"""
A* Planner 

ToDo:
Map file is hardcoaded in the code

"""
import csv
import math
import heapq

import rclpy
from rclpy.node import Node
from robot_msgs.msg import Path,Telemetry
from robot_msgs.srv import PlanPath
from geometry_msgs.msg import Pose

class AStarPlanner(Node):
    def __init__(self, grid_map):
        super().__init__('path_planner')
        self.grid_map = grid_map
        self.rows = len(grid_map)
        self.cols = len(grid_map[0])
        self.robot_id = []

        self.robot_global_goal_publisher_dict = {}

        # mapping for directions later used for traversing the grid in path planner
        self.DIRS = {'^': (-1, 0), '>': (0, 1), 'v': (1, 0), '<': (0, -1)}

        self.plan_path_service_ = self.create_service(PlanPath, 'plan_path', self.plan_path_service_callback)
        self.telemetry_subscriber_ = self.create_subscription(Telemetry, 'telemetry', self.telemetry_callback,1)

        self.create_dynamic_connections_timer_ = self.create_timer(1.0,self.dynamic_connections_callback)

    class Node:
        __slots__ = ('row', 'col', 'cost', 'parent')
        def __init__(self, row, col, cost, parent):
            self.row = row
            self.col = col
            self.cost = cost
            self.parent = parent

        def __repr__(self):
            return f"Node(r={self.row}, c={self.col}, cost={self.cost})"
        def __lt__(self, other):
            # tie-breaker for heap: compare cost
            return self.cost < other.cost

    def telemetry_callback(self, msg : Telemetry):
        if msg.robot_id not in self.robot_id:
            self.robot_id.append(msg.robot_id)

    def dynamic_connections_callback(self):
        # get the current robot id based on telemetry data
        current_robots = set(self.robot_id)

        # get the robot_id whose services are already created
        existing_clients = set(self.robot_global_goal_publisher_dict.keys())
        new_robots      = current_robots - existing_clients

        #create runtime ros service and store it
        for robot_id in new_robots:
            publisher = self.create_publisher(Path,f'{robot_id}/global_path', 10)
            self.get_logger().info(f'New robot discovered {robot_id} successfully created global path publisher ..!')
            self.robot_global_goal_publisher_dict[robot_id] = publisher
    
    def publish_global_path(self,path ,robot_id):
        global_path = Path()
        global_path.header.stamp = self.get_clock().now().to_msg()
        global_path.header.frame_id = 'base_link'
        global_path.robot_id = robot_id

        for i in range(len(path)):
            pose = Pose()
            pose.position.x = float(path[i][0])
            pose.position.y = float(path[i][1])
            pose.position.z = 0.0
            global_path.waypoint.append(pose)
        
        if robot_id in self.robot_global_goal_publisher_dict.keys():
            self.get_logger().info(f'Publishing global path for {robot_id} with {len(global_path.waypoint)} waypoints')
            self.robot_global_goal_publisher_dict[robot_id].publish(global_path)
        else :
            self.get_logger().warning(f'Robot {robot_id} global path publisher not initlised :-) try to call the servce again')
            self.get_logger().info(f'Current robot_id list {self.robot_global_goal_publisher_dict.keys()}')

    def plan_path_service_callback(self, request, response):
        """
        Callback for plan_path sevice
        returns:
        Bool : True if path is found, False otherwise

        """
        start_point = [int(request.start_goal.position.x),int(request.start_goal.position.y)]
        goal_point =  [int(request.end_goal.position.x),int(request.end_goal.position.y)]
        self.get_logger().info(f'Planning path for {request.robot_id} from {start_point} to {goal_point}')
        path = self.plan_path(start_point,goal_point)
        
        if path :
            # convert the python list to robot_msg/Path and publish the waypoints
            self.publish_global_path(path,request.robot_id)
            response.status = True
        else :
            self.get_logger().warning(f'Path not found for {request.robot_id} start point: {request.start_goal} goal_point: {request.end_goal}')
            response.status = False

        return response
        

    def parse_cell(self, cell_str):
        """
        Parse a cell string "(up,right,down,left)"
        into a list of allowed movements.
        """
        if not cell_str:
            return []
        direction_list = cell_str.strip().strip('()').split(',')
        motion_list = []
        
        for direction in direction_list:
            if direction in self.DIRS:
                motion_list.append(self.DIRS[direction])
        return motion_list

    def get_neighbors(self, node):
        """
        Generate neighbor nodes based on one-way allowed directions.
        """
        neighbors = []
        for drow, dcol in self.parse_cell(self.grid_map[node.row][node.col]):
            nrow, ncol = node.row + drow, node.col + dcol
            # check bounds
            if 0 <= nrow < self.rows and 0 <= ncol < self.cols:
                cost = node.cost + math.hypot(drow, dcol)
                neighbors.append(self.Node(nrow, ncol, cost, node))
        return neighbors

    @staticmethod
    def heuristic(point_a, point_b):
        # Manhattan distance for grid
        return abs(point_a.row - point_b.row) + abs(point_a.col - point_b.col)

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append((node.row, node.col))
            node = node.parent
        return list(reversed(path))

    def plan_path(self, start_point, goal_point):
        start = self.Node(start_point[0], start_point[1], 0.0, None)
        goal = self.Node(goal_point[0], goal_point[1], 0.0, None)
        open_dict = {(start.row, start.col): start}
        closed = set()
        heap = [(self.heuristic(start, goal), start)]

        while heap:
            _, current = heapq.heappop(heap)
            key = (current.row, current.col)
            if key in closed:
                continue
            if current.row == goal.row and current.col == goal.col:
                return self.reconstruct_path(current)
            closed.add(key)

            for neighbor in self.get_neighbors(current):
                nkey = (neighbor.row, neighbor.col)
                if nkey in closed:
                    continue
                if nkey not in open_dict or neighbor.cost < open_dict[nkey].cost:
                    open_dict[nkey] = neighbor

                    # f(n) = g(n)+h(n)
                    f = neighbor.cost + self.heuristic(neighbor, goal)
                    heapq.heappush(heap, (f, neighbor))
        return None


def load_csv(path):
    with open(path, newline='') as f:
        return [row for row in csv.reader(f)]

def main(args=None):
    rclpy.init(args=args)

    grid_raw = load_csv("/home/sai/projects/lexxpluss/extras/map.csv")
    # convert grid_raw into grid_map[y][x]dr
    planner = AStarPlanner(grid_raw)
    
    rclpy.spin(planner)


    planner.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
