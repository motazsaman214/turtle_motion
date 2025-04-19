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


    def plot_trajectory(self):
        if not self.x_pose:
            self.get_logger().info("No data to plot.")
            return

        # Extract x and y position data
        times = [point[0] for point in self.x_pose]
        x_positions = [point[1] for point in self.x_pose]
        y_positions = [self.pose.y for _ in self.x_pose]  # Assuming y is constant for simplicity

        # Plot the trajectory
        plt.figure(figsize=(10, 6))
        plt.plot(x_positions, y_positions, label="Turtle Trajectory")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Turtle Trajectory")
        plt.legend()
        plt.grid()
        plt.show()


    def plot_position_vs_time(self):
        if not self.x_pose:
            self.get_logger().info("No data to plot.")
            return

        # Extract time, x, and y position data
        times = [point[0] for point in self.x_pose]
        x_positions = [point[1] for point in self.x_pose]
        y_positions = [self.pose.y for _ in self.x_pose]  # Assuming y is constant for simplicity

        # Plot x and y positions against time
        plt.figure(figsize=(10, 6))
        plt.plot(times, x_positions, label="X Position vs Time")
        plt.plot(times, y_positions, label="Y Position vs Time", linestyle="--")
        plt.xlabel("Time (s)")
        plt.ylabel("Position")
        plt.title("Turtle Position vs Time")
        plt.legend()
        plt.grid()
        plt.show()
    def live_plot(self):
        from matplotlib.animation import FuncAnimation

        # Initialize the plot
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_xlim(0, 12)  # Adjust based on the turtle's environment
        ax.set_ylim(0, 12)  # Adjust based on the turtle's environment
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_title("Live Turtle Trajectory")
        line, = ax.plot([], [], label="Turtle Trajectory")
        ax.legend()
        ax.grid()

        # Update function for the animation
        def update(frame):
            if not self.x_pose:
                return line,
            x_positions = [point[1] for point in self.x_pose]
            y_positions = [self.pose.y for _ in self.x_pose]  # Assuming y is constant for simplicity
            line.set_data(x_positions, y_positions)
            return line,

        # Create the animation
        ani = FuncAnimation(fig, update, interval=100)  # Update every 100ms
        plt.show()

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

# def main(args=None):
#     rclpy.init(args=args)
    
#     turtle_bouncing_node = turtlesim_bouncing_motion()

#     try:
#         # Start the live plot in a separate thread
#         import threading
#         plot_thread = threading.Thread(target=turtle_bouncing_node.live_plot)
#         plot_thread.start()

#         # Spin the node
#         rclpy.spin(turtle_bouncing_node)
#     except KeyboardInterrupt:
#         turtle_bouncing_node.get_logger().info("Shutting down...")

#     rclpy.shutdown()

if __name__ == "__main__":
    main()




