#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import tf_transformations


from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion
from robot_msgs.msg import Telemetry, Path
from robot_msgs.srv import SetState



"""
ToDo:
Add tf publisher the  so That we can visualize the robot pose in rviz
Note: This node will be launched using namespace so that each robot will have its own instance


"""

class WaypointFollower(Node):
    def __init__(self,node_name):

        super().__init__(node_name)

        # robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # controller limits
        self.max_linear_velocity = 2.0   # m/s
        self.max_angular_velocity = 0.09  # rad/s
        self.goal_tolerance = 0.2        # m
        self.yaw_tolerance  = 0.05       # rad

        # waypoint queue
        self.waypoints: list[Pose] = []
        self.current_goal: Pose | None = None
        self.waypoint_status = False
        
        self.create_subscription(Path, 'global_path', self.path_callback, 10)
        #Update the posiiton at 20hz
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)

    def path_callback(self, msg: Path):
        # load up new waypoint list
        self.waypoints = list(msg.waypoint)
        self.current_goal = None
        self.waypoint_status = False
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        self.task_status = False

    def control_loop(self):
        # pick a new goal if needed
        if self.current_goal is None and self.waypoints:
            self.current_goal = self.waypoints.pop(0)
            self.get_logger().info("New goal → "
                f"x={self.current_goal.position.x}, "
                f"y={self.current_goal.position.y}")
            
        elif len(self.waypoints) == 0 and self.current_goal is None:
            self.waypoint_status = True
            self.get_logger().info("No more waypoints to follow",throttle_duration_sec=5.0)
            self.task_status = True

            return

        lin_vel = 0.0
        ang_vel = 0.0

        if self.current_goal:
            # distance to goal from current position
            dist = math.dist((self.current_goal.position.x,self.current_goal.position.y), (self.x, self.y))
            
            # calculate teh slope of the goal
            dx = self.current_goal.position.x - self.x
            dy = self.current_goal.position.y - self.y
            target_yaw = math.atan2(dy, dx) # range of atan2 is [-pi, pi]


            unnormalised_yaw_error = target_yaw - self.theta
            yaw_error =  (unnormalised_yaw_error + math.pi) % (2 * math.pi) - math.pi # normalised to -pi to pi

            if dist > self.goal_tolerance:
                if abs(yaw_error) > self.yaw_tolerance:
                    # Orient the robot to the goal

                    lin_vel = 0.0
                    ang_vel = max(-self.max_angular_velocity,min(self.max_angular_velocity,2.0 * yaw_error))
                    self.get_logger().info(f'Orienting the robot to the goal. yaw_error :{math.degrees(yaw_error)}',throttle_duration_sec=5.0)
                else:
                    # Move towards the goal linearlly
                    forward = min(self.max_linear_velocity, dist)
                    lin_vel = forward * math.cos(yaw_error)
                    ang_vel = max(-self.max_angular_velocity,min(self.max_angular_velocity,2.0 * yaw_error))
                    self.get_logger().info(f'Orienting the robot to the goal. yaw_error :{math.degrees(yaw_error)}',throttle_duration_sec=5.0)

            else:
                self.get_logger().info("Reached current goal")
                self.current_goal = None

        self.x += lin_vel * math.cos(self.theta) * self.dt
        self.y += lin_vel * math.sin(self.theta) * self.dt
        self.theta += ang_vel * self.dt


    def get_robot_pose(self)-> Pose:
        current_pose = Pose()
        current_pose.position.x = self.x
        current_pose.position.y = self.y
        current_pose.position.z = 0.0
        current_orientation = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        type(current_orientation)
        current_pose.orientation.x = current_orientation[0]
        current_pose.orientation.y = current_orientation[1]
        current_pose.orientation.z = current_orientation[2]
        current_pose.orientation.w = current_orientation[3]
        return current_pose
    
    def get_waypoint_status(self) -> bool:
        """
        Returns True if the robot is currently following a waypoint, False otherwise.
        """
        return self.waypoint_status
    
class OdometryPublisher(WaypointFollower):
    def __init__(self):
        super().__init__('odometry_publisher')

        self.telemetry_publisher_ = self.create_publisher(Telemetry, '/telemetry', 10)
        self.robot_id = self.get_namespace().lstrip("/") or "robot_1"
        self.timer_period = 0.2 
        self.robot_state = "idle"
        
        self.timer = self.create_timer(self.timer_period, self.publish_telemetry)
        #######only for simualtion purposes########################
        self.set_robot_state_service = self.create_service(SetState, 'set_robot_state', self.set_robot_state_callback)

    def set_robot_state_callback(self, request, response):
        self.robot_state = request.state
        self.get_logger().info(f"Robot {self.robot_id} state set to: {self.robot_state}")
        return response
    
    def publish_telemetry(self):
        msg = Telemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.robot_id = self.robot_id
        msg.pose = self.get_robot_pose()
        msg.task_status = self.get_waypoint_status()
        msg.robot_state = self.robot_state
        self.telemetry_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
