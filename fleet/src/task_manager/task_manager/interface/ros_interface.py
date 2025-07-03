import rclpy
from rclpy.node import Node
from robot_msgs.msg import Telemetry
from robot_msgs.srv import PlanPath,SetState
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import copy
from abc import abstractmethod


class RosInterface():
    # define all ros related callback functions
    # Make them as abstract methods

    def __init__(self, node_name, **kwargs):
        print(f"Initializing ROS interface with node: {node_name}")
        super().__init__(**kwargs)  # Call next class in MRO
        self.node =  rclpy.create_node(node_name)
        self.robot_data_dict = {}
        self.robot_id = []
        self.initialized_robot_id = []
        self.set_robot_state_client_dict = {}
        self.telemetry_subscriber_ = self.node.create_subscription(Telemetry, 'telemetry', self.telemetry_callback,1)
        self.plan_path_service_client_ = self.node.create_client(PlanPath,'plan_path')
        self.package_drop_publisher_ = self.node.create_publisher(String,'package_drop',1)
        self.create_dynamic_connections_timer_ = self.node.create_timer(1.0,self.dynamic_services_callback)
        self.control_time = 0.1
        self.node.get_logger().info(f'ROS interface initialized Successfully')

    def start_state_machine_control_loop(self):
        self.state_machine_timer = self.node.create_timer(self.control_time,self.tick_state)
        self.node.get_logger().info("Starting State Machine control loop")

    @abstractmethod
    def tick_state(self):
        pass
    
    def publish_package_drop(self,hub_id : str):
        # publish the package drop message to simulate the package drop and write to del hub slot
        
        msg = String()
        msg.data = hub_id
        self.package_drop_publisher_.publish(msg)

    def plan_path_service_response(self,future,robot_id):
        responce = future.result()
        self.node.get_logger().info(f'Path plan for {robot_id} is {responce.status}')

    def set_robot_state(self,robot_id,state):
        robot_state_req = SetState.Request()
        robot_state_req.state = state
        robot_service_future = self.set_robot_state_client_dict[robot_id]
        robot_service_future.call_async(robot_state_req)

    def send_robot2goal(self,start : list ,goal : list,robot_id : str) -> None:
        robot_path_req = PlanPath.Request()
        robot_path_req.start_goal.position.x = float(start[0])
        robot_path_req.start_goal.position.y = float(start[1])
        robot_path_req.end_goal.position.x = float(goal[0])
        robot_path_req.end_goal.position.y = float(goal[1])
        robot_path_req.robot_id = robot_id
        # wait for service and then call
        # wait for service
        if not self.plan_path_service_client_.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('plan_path service not available')
            return
        
        future = self.plan_path_service_client_.call_async(robot_path_req)
        future.add_done_callback(lambda future, rid=robot_id: self.plan_path_service_response(future, rid))

    def dynamic_services_callback(self):
        # get the current robot id based on telemetry data
        current_robots = set(self.robot_data_dict.keys())

        # get the robot_id whose services are already created
        existing_clients = set(self.set_robot_state_client_dict.keys())
        new_robots      = current_robots - existing_clients

        #create runtime ros service and store it
        for robot_id in new_robots:
            client = self.node.create_client(
                SetState,
                f'{robot_id}/set_robot_state'
            )
            self.node.get_logger().info(f'New robot discovered {robot_id} successfully created services ..!')

            self.set_robot_state_client_dict[robot_id] = client
            self.initialized_robot_id.append(robot_id)
    
    def telemetry_callback(self, msg : Telemetry):
        if msg.robot_id not in self.robot_id:
            self.robot_id.append(msg.robot_id)

        self.robot_data_dict[msg.robot_id] = msg

    def get_robot_telemetry(self,robot_id: str) -> Telemetry:

        if robot_id in self.robot_data_dict:
            return copy.deepcopy(self.robot_data_dict[robot_id])
        
        return None

    def plan_path(self,robot_id: str, start_goal : Pose, end_goal: Pose) :
        plan_path_request = PlanPath.Request()
        plan_path_request.robot_id = robot_id
        plan_path_request.start_goal = start_goal
        plan_path_request.end_goal = end_goal
        while not self.plan_path_service_client_.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

        future = self.plan_path_service_client_.call_async(plan_path_request)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                response = future.result() # respince is bool
                if response.status:
                    self.node.get_logger().info(f"Sending robot_{robot_id} from : {start_goal} to {end_goal}")
                else:
                    self.node.get_logger().error(f"Failed to plan path for robot_{robot_id} from : {start_goal} to {end_goal}")
            except Exception as e:
                self.node.get_logger().error(f"Service call failed: {e}")
    