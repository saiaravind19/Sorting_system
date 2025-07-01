import rclpy
from rclpy.node import Node

from robot_msgs.msg import Telemetry
from std_msgs.msg import String
from task_manager.state_machine.robot_statemachine import PickupRobot

# task manager communicate with communication hub over grpc
# currently running a single threaded execuitor
# Can use mult threaded execuitor but need to handle race conditions
    
class TaskManager(PickupRobot):

    def __init__(self):
        super().__init__('task_manager')
        self.start_state_machine_control_loop()


#########add service to start and stop the system#############
###stop the control loop 


def main(args=None):
    rclpy.init(args=args)

    task_manager = TaskManager()
    rclpy.spin(task_manager.node)
    # Destroy the node explicitly
    task_manager.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()