"""
GoToGoal ROS2 Node

This node controls a turtlesim turtle to move towards a specified goal pose.

Subscriptions:
    /turtle1/pose (turtlesim.msg.Pose): Receives the current pose of the turtle.
    /goal_pose (turtlesim.msg.Pose): Receives the target goal pose for the turtle.

Publications:
    /turtle1/cmd_vel (geometry_msgs.msg.Twist): Publishes velocity commands to move the turtle.

Functionality:
    - Computes the distance and angle to the goal.
    - Rotates the turtle towards the goal before moving forward.
    - Stops the turtle when the goal is reached within a threshold.
    - Logs when a new goal is received and when the goal is reached.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class GoToGoal(Node):

    def __init__(self):
        super().__init__('Go_To_Goal')
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/goal_pose', self.goal_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_goal)

        self.pose = Pose()
        self.goal = None
        self.goal_reached = False

    def pose_callback(self, msg):
        self.pose = msg

    def goal_callback(self, msg):
        self.goal = msg
        self.goal_reached = False
        self.get_logger().info(f'New Goal Received: x={msg.x}, y={msg.y}, theta={msg.theta}')

    def go_to_goal(self):
        if self.goal is None or self.goal_reached:
            return

        velocity = Twist()

        distance = math.sqrt((self.goal.x - self.pose.x) ** 2 + (self.goal.y - self.pose.y) ** 2)
        angle_to_goal = math.atan2((self.goal.y - self.pose.y), (self.goal.x - self.pose.x))
        angle_error = angle_to_goal - self.pose.theta

        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        distance_threshold = 0.1
        angle_threshold = 0.01
        kp = 6.0

        if abs(angle_error) > angle_threshold:
            velocity.angular.z = kp * angle_error
        elif distance >= distance_threshold:
            velocity.linear.x = kp * distance
        else:
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self.goal_reached = True
            self.get_logger().info("Goal Reached")

        self.vel_pub.publish(velocity)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()