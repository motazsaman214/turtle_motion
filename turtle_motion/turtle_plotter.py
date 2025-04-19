"""
turtle_plotter ROS2 Node

This node subscribes to the pose of a turtlesim turtle and provides a live plot of the turtle's x and y positions over time.
It uses matplotlib in interactive mode to update the plots in real time.

Subscriptions:
    /turtle1/pose (turtlesim.msg.Pose): Receives the current pose of the turtle.

Functionality:
    - Records the x and y positions and timestamps as the turtle moves.
    - Plots x and y positions versus time in real time using matplotlib.
    - Uses threading to run the plotting loop without blocking ROS2 callbacks.
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import matplotlib.pyplot as plt
import threading
import time

class TurtleLivePlotter(Node):
    def __init__(self):
        super().__init__('turtle_live_plotter')
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.x_data = []
        self.y_data = []
        self.t_data = []
        self.start_time = time.time()
        self.lock = threading.Lock()
        threading.Thread(target=self.plot_thread, daemon=True).start()

    def pose_callback(self, msg):
        with self.lock:
            self.x_data.append(msg.x)
            self.y_data.append(msg.y)
            self.t_data.append(time.time() - self.start_time)

    def plot_thread(self):
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1)
        while rclpy.ok():
            with self.lock:
                ax1.clear()
                ax2.clear()
                ax1.plot(self.t_data, self.x_data, label='x')
                ax2.plot(self.t_data, self.y_data, label='y', color='orange')
                ax1.set_ylabel('X Position')
                ax2.set_ylabel('Y Position')
                ax2.set_xlabel('Time (s)')
                ax1.legend()
                ax2.legend()
            plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleLivePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()