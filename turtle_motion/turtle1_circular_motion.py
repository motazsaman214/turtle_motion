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
            self.get_logger().warn("Usage: python circular_motion.py <linear_velocity> <radius>")
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
