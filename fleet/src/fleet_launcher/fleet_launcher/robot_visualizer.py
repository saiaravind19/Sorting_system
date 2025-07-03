#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from robot_msgs.msg import Telemetry

import matplotlib.pyplot as plt
from itertools import cycle

class PoseVisualizer(Node):
    def __init__(self):
        super().__init__('pose_visualizer_points')
        # store latest telemetry per robot_id: x, y, state, task
        self.poses = {}  # { robot_id: (x, y, state, task) }
        # assign a distinct color per robot
        self._colors = {}
        self._color_cycle = cycle(plt.rcParams['axes.prop_cycle'].by_key()['color'])

        # subscriber
        self.subscription = self.create_subscription(
            Telemetry,
            'telemetry',
            self.telemetry_cb,
            10
        )

        self.backgrodund_img = plt.imread('/home/sai/projects/lexxpluss/extras/layout.png')
        # set up Matplotlib
        plt.ion()
        
        self.fig, self.ax = plt.subplots()
        self.ax.grid(True)
        self.ax.imshow(self.backgrodund_img,
                       extent=[0.0, 35, 0.2, 7],
                       aspect='equal',
                       zorder=0)    
        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlim(-0.2, 36)
        self.ax.set_ylim(-0.2, 8)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Robot Positions (Live)')

        # timer for periodic update
        self.timer = self.create_timer(0.1, self.update_plot)

    def telemetry_cb(self, msg: Telemetry):
        # extract x, y
        x = msg.pose.position.x
        y = msg.pose.position.y
        # store
        self.poses[msg.robot_id] = (x, y, msg.robot_state, msg.task_status)
        # assign color if new robot_id
        if msg.robot_id not in self._colors:
            self._colors[msg.robot_id] = next(self._color_cycle)

    def update_plot(self):
        self.ax.cla()
        self.ax.grid(True)
        self.ax.imshow(self.backgrodund_img,
                       extent=[-0.5, 35, 0.0, 7],
                       aspect='equal',
                       zorder=0) 
        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlim(-0.2, 36)
        self.ax.set_ylim(-0.2, 8)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Robot Positions (Live)')

        # plot each robot as a colored point
        for robot_id, (x, y, state, task) in self.poses.items():
            color = self._colors.get(robot_id, 'k')
            self.ax.scatter(x, y, s=50, color=color, label=robot_id)
            # label with state and task
            self.ax.text(
                x + 0.1, y + 0.1,
                f"{state}",
                fontsize=8,
                color=color
            )

        # show legend outside plot
        if self.poses:
            self.ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1))

        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = PoseVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()

if __name__ == '__main__':
    main()
