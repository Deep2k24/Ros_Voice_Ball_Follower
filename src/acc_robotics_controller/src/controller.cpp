#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class DifferentialDriveController : public rclcpp::Node
{
public:
    DifferentialDriveController() : Node("differential_drive_controller")
    {
        // Declare and initialize parameters
        this->declare_parameter<double>("wheelbase", 0.09);  // Default: 0.45 meters
        this->declare_parameter<double>("wheel_radius", 0.025);  // Default: 0.1 meters
        this->declare_parameter<double>("max_rpm", 100.0);    // Default: 100 RPM

        // Get parameter values
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_rpm_ = this->get_parameter("max_rpm").as_double();

        // Publishers for left and right wheel RPMs
        left_wheel_rpm_publisher_ = this->create_publisher<std_msgs::msg::Float64>("left_wheel_rpm", 10);
        right_wheel_rpm_publisher_ = this->create_publisher<std_msgs::msg::Float64>("right_wheel_rpm", 10);

        // Subscriber to /cmd_vel
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Differential Drive Controller Node Started");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extract linear and angular velocities from the /cmd_vel message
        double linear_velocity = msg->linear.x;  // Forward velocity (m/s)
        double angular_velocity = msg->angular.z;  // Rotational velocity (rad/s)

        // Compute wheel velocities (m/s) for a differential drive robot
        double left_wheel_velocity = linear_velocity - (angular_velocity * wheelbase_ / 2.0);
        double right_wheel_velocity = linear_velocity + (angular_velocity * wheelbase_ / 2.0);

        // Convert wheel velocities to RPM
        double left_wheel_rpm = (left_wheel_velocity / (2.0 * M_PI * wheel_radius_)) * 60.0;
        double right_wheel_rpm = (right_wheel_velocity / (2.0 * M_PI * wheel_radius_)) * 60.0;

        // Clamp RPM values to the maximum allowed RPM
        left_wheel_rpm = std::max(-max_rpm_, std::min(left_wheel_rpm, max_rpm_));
        right_wheel_rpm = std::max(-max_rpm_, std::min(right_wheel_rpm, max_rpm_));

        // Publish the RPM values
        auto left_rpm_msg = std_msgs::msg::Float64();
        left_rpm_msg.data = left_wheel_rpm;
        left_wheel_rpm_publisher_->publish(left_rpm_msg);

        auto right_rpm_msg = std_msgs::msg::Float64();
        right_rpm_msg.data = right_wheel_rpm;
        right_wheel_rpm_publisher_->publish(right_rpm_msg);

        RCLCPP_INFO(this->get_logger(), "Published RPMs - Left: %.2f RPM, Right: %.2f RPM", left_wheel_rpm, right_wheel_rpm);
    }

    // ROS 2 publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_rpm_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_rpm_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    // Parameters
    double wheelbase_;
    double wheel_radius_;
    double max_rpm_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DifferentialDriveController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}