import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class GoalNavigation(Node):
    def __init__(self):
        super().__init__('goal_navigation')

        # Declare parameters for the goal
        self.declare_parameter('goal_x', 2.0)  # X-coordinate of the goal
        self.declare_parameter('goal_y', 2.0)  # Y-coordinate of the goal

        # Get the goal coordinates from parameters
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        # Create publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriber for odometry data
        self.odom_subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize robot's current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # PID gains for angular velocity
        self.kp_angular = 0.8  # Reduced proportional gain for smoother turns
        self.kd_angular = 0.05  # Derivative gain to reduce overshooting
        self.previous_yaw_error = 0.0

        # PID gains for linear velocity
        self.kp_linear = 0.4  # Reduced linear gain for smoother approach

        # Flag to indicate if the goal is reached
        self.goal_reached = False

    def odom_callback(self, msg):
        # Extract the robot's current position and orientation from odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Navigate to the goal
        self.navigate_to_goal()

    def navigate_to_goal(self):
        if self.goal_reached:
            return

        # Compute the distance to the goal
        distance = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

        # Compute the angle to the goal
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

        # Compute the angular error
        yaw_error = angle_to_goal - self.current_yaw

        # Normalize the yaw error to the range [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # Stop the robot if the goal is reached
        if distance < 0.15:  # Reduced threshold for more precise stopping
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            self.goal_reached = True
            return

        # Create a Twist message to control the robot
        msg = Twist()

        # Rotate the robot to face the goal
        if abs(yaw_error) > 0.1:  # Threshold for angular alignment
            angular_velocity = self.kp_angular * yaw_error + self.kd_angular * (yaw_error - self.previous_yaw_error)
            angular_velocity = max(min(angular_velocity, 0.8), -0.8)  # Clamp angular velocity to [-0.8, 0.8]
            msg.angular.z = angular_velocity  # PID control for angular velocity
            msg.linear.x = 0.0  # Stop linear movement while aligning
        else:
            # Move towards the goal
            linear_velocity = self.kp_linear * distance
            linear_velocity = max(min(linear_velocity, 0.3), 0.0)  # Clamp linear velocity to [0.0, 0.3]
            msg.linear.x = linear_velocity  # Proportional control for linear velocity
            msg.angular.z = 0.0  # No angular velocity once aligned

        # Update the previous yaw error for derivative control
        self.previous_yaw_error = yaw_error

        # Log the velocity command
        self.get_logger().info(f"Publishing cmd_vel: linear_x={msg.linear.x:.2f}, angular_z={msg.angular.z:.2f}")

        # Publish the velocity command
        self.publisher_.publish(msg)

    def stop_robot(self):
        # Publish a zero velocity command to stop the robot
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()