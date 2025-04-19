"""
turtle_bouncing_motion ROS2 Node

This node controls a turtlesim turtle to move horizontally back and forth ("bouncing") between the left and right walls.
It publishes velocity commands to move the turtle and reverses direction when the turtle reaches the boundaries.
The node also records the turtle's x position over time and provides plotting functions to visualize the trajectory and position vs. time.

Subscriptions:
    /turtle1/pose (turtlesim.msg.Pose): Receives the current pose of the turtle.

Publications:
    /turtle1/cmd_vel (geometry_msgs.msg.Twist): Publishes velocity commands to move the turtle.

Functionality:
    - Moves the turtle right until it reaches the right wall, then reverses direction.
    - Records x position and time for plotting.
    - Provides methods to plot the trajectory and position vs. time.
    - Includes a live plot option for real-time visualization.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import matplotlib.pyplot as plt


class turtlesim_bouncing_motion(Node):
    def __init__(self):
        super().__init__("turtlesim_bouncing_motion")

        self.vel_pub_  = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        self.timer = self.create_timer(0.001, self.bouncing)

        self.move_right = True
        self.pose = None
        self.time=0
        self.time_range= []
        self.x_pose=[]

        self.pose_x = 0.0

    def bouncing(self):
        vel = Twist()
        vel.linear.x = 20.0 if self.move_right else -20.0
        self.vel_pub_.publish(vel)
            
    def pose_callback(self, msg):
        self.time+=0.01
        self.pose= msg
        self.pose_x = msg.x
        self.x_pose.append([self.time,self.pose_x])
        if self.pose_x >= 11:
            self.move_right = False
        elif self.pose_x <= 0.1:
            self.move_right = True


def main(args=None):
    rclpy.init(args=args)
    
    turtlesim_bouncing_node = turtlesim_bouncing_motion()

    try:
        rclpy.spin(turtlesim_bouncing_node)
    except KeyboardInterrupt:
        turtlesim_bouncing_node.get_logger().info("Shutting down...")

    turtlesim_bouncing_node.plot_trajectory()
    turtlesim_bouncing_node.plot_position_vs_time()
    rclpy.shutdown()


if __name__ == "__main__":
    main()