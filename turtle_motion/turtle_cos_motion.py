"""
turtle_cos_motion ROS2 Node

This node controls a turtlesim turtle to follow a cosine-shaped trajectory along the x-axis.
It uses a service call to teleport the turtle to the initial position, then publishes velocity commands to move the turtle along the path defined by y = 5.6 + cos(x).

Subscriptions:
    /turtle1/pose (turtlesim.msg.Pose): Receives the current pose of the turtle.

Publications:
    /turtle1/cmd_vel (geometry_msgs.msg.Twist): Publishes velocity commands to move the turtle.

Services:
    /turtle1/teleport_absolute (turtlesim.srv.TeleportAbsolute): Used to set the turtle's initial position.

Functionality:
    - Teleports the turtle to the starting point (x=0.0, y=5.6, theta=0.0).
    - Generates a set of points along the cosine curve.
    - Moves the turtle sequentially through each point using proportional control.
    - Stops when the trajectory is completed.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from math import sin, atan2, cos, sqrt

class turtle_cos_motion(Node):
    def __init__(self):
        super().__init__('turtle_cos_motion')
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.timer = self.create_timer(0.0001, self.timer_callback)
        self.x = None
        self.y = None
        self.theta = 0.0
        self.index = 0
        self.point = self.create_points()
        self.initial_pose_set = False

    def create_points(self):
        point = []
        x = 0
        for i in range(109):
            point.append((x, 5.6 + cos(x)))
            x += 0.1
        return point

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def timer_callback(self):
        if not self.initial_pose_set:
            if not self.teleport_client.wait_for_service(timeout_sec=1.0):
                return
            request = TeleportAbsolute.Request()
            request.x = 0.0
            request.y = 5.6
            request.theta = 0.0
            self.teleport_client.call_async(request)
            self.initial_pose_set = True
            return

        if self.x is None or self.y is None:
            return  

        if self.index >= len(self.point):
            return

        goal_x, goal_y = self.point[self.index]
        distance = sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)
        angle_to_goal = atan2((goal_y - self.y), (goal_x - self.x))
        angle_error = atan2(sin(angle_to_goal - self.theta), cos(angle_to_goal - self.theta))

        velocity = Twist()
        kp = 60.0
        distance_threshold = 0.0001
        angle_threshold = 0.005

        if abs(angle_error) > angle_threshold:
            velocity.angular.z = kp * angle_error
        elif distance > distance_threshold:
            velocity.linear.x = kp * distance
        else:
            self.index += 1

        self.vel_pub.publish(velocity)

def main(args=None):
    rclpy.init(args=args)
    node = turtle_cos_motion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()