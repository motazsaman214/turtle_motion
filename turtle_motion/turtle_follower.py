import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import math


class TurtleFollower(Node):

    def __init__(self):
        super().__init__('TurtleFollower')
        self.vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.goal_pos_sub = self.create_subscription(Pose, '/turtle1/pose', self.goal_pose_callback, 10)
        self.my_pos_sub = self.create_subscription(Pose, '/turtle2/pose', self.my_pose_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.goal_pose = Pose()
        self.my_pose = Pose()  # Initialize my_pose attribute

    def goal_pose_callback(self, data):
        self.goal_pose = data

    def my_pose_callback(self, data):
        self.my_pose = data

    def go_to_goal(self):
        velocity = Twist()

        # Extract goal and current pose coordinates
        goal_x = self.goal_pose.x
        goal_y = self.goal_pose.y
        current_x = self.my_pose.x
        current_y = self.my_pose.y

        # Calculate distance and angle to goal
        distance_to_goal = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)

        # Define thresholds and control parameters
        distance_threshold = 0.9
        angle_threshold = 0.01
        kp = 4

        # Calculate angle error
        angle_error = angle_to_goal - self.my_pose.theta

        # Proportional control
        if abs(angle_error) > angle_threshold:
            velocity.angular.z = kp * angle_error
        elif distance_to_goal >= distance_threshold:
            velocity.linear.x = kp * distance_to_goal
        else:
            velocity.linear.x = 0.0
            self.get_logger().info("Goal Reached")

        # Publish velocity command
        self.vel_pub.publish(velocity)


def main(args=None):
    rclpy.init(args=args)
    turtle_follower = TurtleFollower()
    rclpy.spin(turtle_follower)
    turtle_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
