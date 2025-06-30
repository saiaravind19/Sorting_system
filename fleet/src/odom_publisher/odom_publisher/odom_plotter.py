#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_msgs.msg import Telemetry   
import matplotlib.pyplot as plt
import numpy as np
import math

class TelemetryPlotter(Node):
    def __init__(self):
        super().__init__('telemetry_plotter')
        self.xs = []
        self.ys = []
        self.headings = []

        # interactive matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        self.scatter = self.ax.scatter([], [], s=50)
        self.quiver = self.ax.quiver([], [], [], [], angles='xy', scale_units='xy', scale=1)

        self.ax.set_title("Live Robot Position")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)
        self.ax.set_aspect('equal', 'box')

        # subscribe
        self.create_subscription(
            Telemetry,
            'telemetry',
            self.telemetry_cb,
            10)

    def telemetry_cb(self, msg):
        # extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.xs.append(x)
        self.ys.append(y)

        # extract yaw from quaternion
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self.headings.append(yaw)

        # update scatter
        self.scatter.set_offsets(np.column_stack((self.xs, self.ys)))

        # update quiver arrows (last point only)
        # you can plot arrow at each point by passing full arrays
        U = np.cos(self.headings)
        V = np.sin(self.headings)
        self.quiver.set_offsets(np.column_stack((self.xs, self.ys)))
        self.quiver.set_UVC(U, V)

        # autoscale
        self.ax.relim()
        self.ax.autoscale_view()

        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPlotter()
    node.get_logger().info("TelemetryPlotter started, listening on 'telemetry'")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
