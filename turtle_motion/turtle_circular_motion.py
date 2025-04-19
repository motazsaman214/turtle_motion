"""
turtle_circular_motion ROS2 Node

This node controls a turtlesim turtle to move in a circular path.
It publishes velocity commands to the turtle based on user-specified linear velocity and radius.

Usage:
    ros2 run turtle_motion turtle_circular_motion <linear_velocity> <radius>

Arguments:
    linear_velocity (float): The desired linear velocity of the turtle.
    radius (float): The radius of the circular path.

Publications:
    /turtle1/cmd_vel (geometry_msgs.msg.Twist): Publishes velocity commands to move the turtle.

Functionality:
    - Reads linear velocity and radius from command-line arguments.
    - Computes the required angular velocity for circular motion.
    - Publishes velocity commands to move the turtle in a circle.
    - Logs warnings if arguments are missing or invalid.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys


class CircularMotion(Node):
    def __init__(self):
        super().__init__("circular_motion")
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.motion_callback)  # Change timer interval to 1.0 second

    def motion_callback(self):
        velocity = Twist()

        # Check if the required command-line arguments are provided
        if len(sys.argv) < 3:
            self.get_logger().warn("Usage: ros2 run turtle_motion turtle_circular_motion <linear_velocity> <radius>")
            return
        
        try:
            linear_vel = float(sys.argv[1])
            radius = float(sys.argv[2])
        except ValueError:
            self.get_logger().error("Invalid argument type. Both arguments must be numbers.")
            return

        velocity.linear.x = linear_vel
        velocity.linear.y = 0.0 # Keep linear y velocity zero for a 2D motion

        # Compute angular velocity for circular motion (angular speed = linear speed / radius)
        velocity.angular.z = linear_vel / radius

        self.publisher.publish(velocity)
        self.get_logger().info("Moving turtle with linear_x={}, angular_z={}".format(
            velocity.linear.x, velocity.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = CircularMotion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()