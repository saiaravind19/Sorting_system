from task_manager.interface.ros_interface import RosInterface
from robot_msgs.srv import SetState
import copy
from task_manager.utils.map_utils import map_utils

"""
Check if can use multiprocessing 

Cases like decommission the robot is not handled so robot will block the grid if there is any error

"""

class PickupRobot(RosInterface): # rpc class inheritance to be added
    
    def __init__(self,node_name):
        super().__init__(node_name)
        self.map_utils = map_utils('/home/sai/projects/lexxpluss/dump_grid_config.json')
        self._last_stamp = {}

    def tick_state(self):
        for robot_id in self.initialized_robot_id:
            robot_telem = self.get_robot_telemetry(robot_id)

            # Avoid ticking state machine on same robot_state this is to ensure latest data is processed
            stamp = robot_telem.header.stamp.sec + robot_telem.header.stamp.nanosec * 1e-2
            if self._last_stamp.get(robot_id) == stamp:
                continue  
            self._last_stamp[robot_id] = stamp

            if robot_telem.task_status == True:
                current_position= [round(robot_telem.pose.position.x),round(robot_telem.pose.position.y)]
                if robot_telem.robot_state == "idle":
                    # get the available queue grid
                    # choose the best queue based on manhatten distance
                    # block the queuing grid
                    # set the state to robot 2 queue
                    queueing_grid = self.map_utils.get_nearest_available_grid(current_position,"queuing_grid")
                    if queueing_grid is not None:
                        # set the state to robot2queue  -> call the service
                        # block queuing grid
                        # send the path to path planner over sevice call
                        self.set_robot_state(robot_id,"robot2queue")
                        self.node.get_logger().info(f'Sending {robot_id} to Queuing grid {queueing_grid}')
                        self.map_utils.block_queuing_grid(queueing_grid,robot_id)
                        self.send_robot2goal(current_position,queueing_grid,robot_id)

                    else :
                        self.node.get_logger().warning("no available queue grid",throttle_duration_sec=5.0)

                elif robot_telem.robot_state == "robot2queue":

                    # check monitor the path completetion - > task status
                    # if task completed
                        # get corresponding feeding grid
                        # set the state to robot 2 feeder
                    feeding_grid = self.map_utils.get_nearest_available_grid(current_position,"feeding_grid")
                    if feeding_grid is not None:
                        self.set_robot_state(robot_id,"robot2feeder")
                        self.node.get_logger().info(f'Sending {robot_id} to Queuing grid {feeding_grid}')

                        self.map_utils.block_feeding_grid(feeding_grid,robot_id)
                        self.send_robot2goal(current_position,feeding_grid,robot_id)
                        self.map_utils.unblock_queuing_grid(current_position)
                    else :
                        self.node.get_logger().warning("no available feeding grid",throttle_duration_sec=5.0)

                elif robot_telem.robot_state == "robot2feeder":
                    # check monitor the path completetion - > task status
                    # if task completed
                    # Means robot is at feeding grid - > free the queue grid
                    # Request for parcel or check for del hub status
                    # accepted set the state to robto2delhub
                    # else pas

##################################check for status from delivery hub  ######################

                    dumping_grid = self.map_utils.get_respective_dumping_grid(current_position)
                    if dumping_grid is not None:
                        self.set_robot_state(robot_id,"robot2delhub")
                        self.node.get_logger().info(f'Sending {robot_id} to dumping grid {dumping_grid}')

                        self.map_utils.block_dumping_grid(dumping_grid,robot_id)
                        self.send_robot2goal(current_position,dumping_grid,robot_id)
                        self.map_utils.unblock_feeding_grid(current_position)
                        
                    else :
                        self.node.get_logger().warning("no available dumping grid",throttle_duration_sec=5.0)


                elif robot_telem.robot_state == "robot2delhub":
                    # check monitor the path completetion - > task status
                    # requets for package acceptance
                    # once accepted set the state to idel again
                    # unblock the dumping grid
############################ wait for package to be accepted ######################

                    self.set_robot_state(robot_id,"idle")
                    self.node.get_logger().info(f'Robot finished {robot_id} to complete cycle :-) setting state to idle')

                    self.map_utils.unblock_dumping_grid(current_position)

                else : 
                    self.node.get_logger().warning(f'unknown state detected for {robot_id} state : {robot_telem.robot_state}',throttle_duration_sec=5.0)

                




