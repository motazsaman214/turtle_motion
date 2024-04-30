# turtle_motion: ROS Nodes for Turtlebot Control

This repository contains ROS (Robot Operating System) nodes for controlling a simulated turtlebot using the turtlesim package. The nodes provide functionalities for:

- **Goal Following (`turtle_follower`):**
  - Enables a turtle (e.g., `turtle2`) to follow another turtle (e.g., `turtle1`).
  - Subscribes to the pose of the target turtle (`/turtle1/pose`).
  - Calculates the distance and angle between the following turtle and the target turtle.
  - Implements a proportional control strategy to adjust the linear and angular velocities of the following turtle, guiding it towards the target.
  - Stops the movement and logs a message upon reaching the goal within a specified threshold.

- **Circular Motion (`turtle1_circular_motion`):**
  - Allows a turtle (e.g., `turtle1`) to perform circular motion.
  - Takes two command-line arguments: linear velocity and radius.
  - Validates user input to ensure numeric values are provided.
  - Sets the linear x velocity of the turtle to the provided value and maintains zero linear y velocity for a 2D motion.
  - Calculates the angular velocity based on the formula `angular_velocity = linear_velocity / radius` to achieve the desired circular motion.
  - Publishes the calculated twist message (`/turtle1/cmd_vel`) to control the turtle's movement.
  - Logs information about the applied linear and angular velocities for debugging or monitoring purposes.

- **Go to Goal (`GoToGoal`):**
  - Enables a turtle (e.g., `turtle1`) to navigate to a specified goal location.
  - Takes three command-line arguments: desired x, y coordinates, and target orientation (theta) of the goal pose.
  - Subscribes to the current pose of the turtle (`/turtle1/pose`).
  - Calculates the distance and angle between the current and goal poses.
  - Employs a proportional control approach to determine the linear and angular velocities for movement.
  - Stops the movement and logs a "Goal Reached" message upon reaching the goal within predefined thresholds for distance and angle.

## Getting Started

1. Clone this repository to your local machine.
2. Install ROS dependencies according to your ROS installation.
3. Source your ROS environment (`source /opt/ros/foxy/setup.bash` for Foxy or similar for your ROS version).
4. Navigate to the repository directory in your terminal.
5. Build the package:
    ```bash
    cd turtle_motion
    colcon build --symlink-install
    ```

## Running the Nodes

To run the nodes:

1. Open a new terminal and source the ROS environment.
2. In the second terminal, navigate to the repository directory and run the desired node using the following commands (replace `<goal_x>`, `<goal_y>`, `<goal_theta>`, `<linear_velocity>`, and `<radius>` with the appropriate values):

    ```bash
    ros2 run turtle_motion turtle_follower
    ros2 run turtle_motion turtle1_circular_motion <linear_velocity> <radius>
    ros2 run turtle_motion GoToGoal <goal_x> <goal_y> <goal_theta>
    ```

Replace `<goal_x>`, `<goal_y>`, `<goal_theta>`, `<linear_velocity>`, and `<radius>` with the specific values you want to use for your simulations.
