import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import math


class GoToGoal(Node):

    def __init__(self):
        super().__init__('Go_To_Goal')
        self.vel_pub = self.create_publisher( Twist , '/turtle1/cmd_vel', 10)
        self.pos_sub = self.create_subscription( Pose ,'/turtle1/pose', self.pose_callback ,10 )
        self.create_timer = self.create_timer(0.1,self.go_to_goal)
        self.pose=Pose()

    def pose_callback(self,data):
        
        self.pose= data

    def go_to_goal(self):
        goal = Pose()

        goal.x = float(sys.argv[1])
        goal.y = float(sys.argv[2])

        goal.theta=float(sys.argv[3])

        velocity= Twist()

        distance_to_goal = math.sqrt( (goal.x - self.pose.x )**2 + (goal.y - self.pose.y)**2)
        angel_to_goal = math.atan2( (goal.y - self.pose.y) , (goal.x - self.pose.y) )

        distance_threeshold = 0.1
        angel_threeshold = 0.01

        angel_error = angel_to_goal - self.pose.theta

        kp=6

        if abs(angel_error)>angel_threeshold:
            velocity.angular.z = kp * angel_error
        else:
            if distance_to_goal >= distance_threeshold:
                velocity.linear.x = kp * distance_to_goal
            else:
                velocity.linear.x = 0.0

                self.get_logger().info("Goal Reached ")
                quit()
        self.vel_pub.publish(velocity)




def main(args=None):
    rclpy.init(args=args)
    go_to_goal = GoToGoal()
    rclpy.spin(go_to_goal)
    go_to_goal.destroy_node()
    rclpy.shutdown()



if __name__ == 'main':

    main()