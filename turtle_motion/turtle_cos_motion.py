# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from turtlesim.msg import Pose
# from math import sin, atan2, cos, sqrt

# class turtle_sinusoidal_motion(Node):
#     def __init__(self):
#         super().__init__('turtle_sinusoidal_motion')
#         self.vel_pub  = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
#         self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
#         self.timer    = self.create_timer(0.01, self.timer_callback)
#         self.x = 0.0
#         self.y = 5.6
#         self.theta = 0.0
#         self.index = 0
#         self.point = self.creat_points()

#     def go_to_initial_position(self):
#         self.x = 0.0
#         self.y = 5.6
#         self.theta = 0.0
#         self.index = 0
#         if self.goal is None or self.goal_reached:
#             return

#         velocity = Twist()

#         distance = math.sqrt((self.goal.x - self.pose.x) ** 2 + (self.goal.y - self.pose.y) ** 2)
#         angle_to_goal = math.atan2((self.goal.y - self.pose.y), (self.goal.x - self.pose.x))
#         angle_error = angle_to_goal - self.pose.theta

#         angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

#         distance_threshold = 0.1
#         angle_threshold = 0.01
#         kp = 6.0

#         if abs(angle_error) > angle_threshold:
#             velocity.angular.z = kp * angle_error
#         elif distance >= distance_threshold:
#             velocity.linear.x = kp * distance
#         else:
#             velocity.linear.x = 0.0
#             velocity.angular.z = 0.0
#             self.goal_reached = True
#             self.get_logger().info("Goal Reached")

#         self.vel_pub.publish(velocity)


#     def creat_points(self):
#         point = []
#         x = 0
#         y = 0
#         for i in range(110):
#             point.append((x, y))
#             x += 0.1
#             y = 5.6 + sin(x)
#         return point

#     def pose_callback(self, msg):
#         self.x = msg.x
#         self.y = msg.y
#         self.theta = msg.theta

#     def timer_callback(self):
#         if self.index >= len(self.point):
#             return
#         goal_x, goal_y = self.point[self.index]
#         distance = sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)
#         angle_to_goal = atan2((goal_y - self.y), (goal_x - self.x))
#         angle_error = atan2(sin(angle_to_goal - self.theta), cos(angle_to_goal - self.theta))
#         velocity = Twist()
#         kp = 6.0
#         distance_threshold = 0.1
#         angle_threshold = 0.05
#         if abs(angle_error) > angle_threshold:
#             velocity.angular.z = kp * angle_error
#         elif distance > distance_threshold:
#             velocity.linear.x = kp * distance
#         else:
#             self.index += 1
#         self.vel_pub.publish(velocity)

# def main(args=None):
#     rclpy.init(args=args)
#     node = turtle_sinusoidal_motion()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()






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
