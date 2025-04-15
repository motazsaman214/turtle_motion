import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math 
import matplotlib as plt


class turtlesim_bouncing(Node):
    def __init__(self):
        super().__init__("turtlesim_bouncing")

        self.vel_pub_  = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_pub_ = self.create_publisher(Pose, "/turtle1/pose", 10)
        self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        self.timer = self.create_timer(0.01, self.bouncing)

        self.move_right = True

    def bouncing(self):
        vel = Twist()

        vel.linear.x = 1.0 if self.move_right else -1.0
        self.vel_pub_.publish(vel)
            


    def pose_callback(self, msg):
        self.pose= msg
        self.pose_x = msg.x
        self.pose_y = msg.y
        if self.pose_x >= 11:
            self.move_right = False
        elif self.pose_x <= -0.03:
            self.move_right = True

        














def main(args=None):
    

    rclpy.init(args=args)
    
    turtlesim_bouncing_node = turtlesim_bouncing()

    rclpy.spin(turtlesim_bouncing_node)
    rclpy.shutdown()







if __name__ == "__main__":
    main()