#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <memory>
#include <iostream>

// Wheel parameters
const double WHEEL_TO_WHEEL_DISTANCE = 0.443;  // meters
const double WHEEL_DIAMETER = 0.181;  // meters
const double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;  // meters

struct WheelData {
    double left_wheel_rpm;
    double right_wheel_rpm;
    double target_linear_velocity;
    double target_angular_velocity;
    uint64_t timestamp_ms;
};

class WheelVelocityNode : public rclcpp::Node {
public:
    WheelVelocityNode() : Node("wheel_velocity_node") {
        // Create publisher for Wheel Data
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WheelVelocityNode::calculate_and_publish, this));

        // Create subscriber to cmd_vel
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&WheelVelocityNode::cmd_vel_callback, this, std::placeholders::_1)
        );
    }

private:
    // Subscription callback for cmd_vel
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_linear_velocity_ = msg->linear.x;
        target_angular_velocity_ = msg->angular.z;
    }

    // Calculate wheel velocities and publish
    void calculate_and_publish() {
        double v_left = (target_linear_velocity_ - (WHEEL_TO_WHEEL_DISTANCE / 2) * target_angular_velocity_) / WHEEL_RADIUS;
        double v_right = (target_linear_velocity_ + (WHEEL_TO_WHEEL_DISTANCE / 2) * target_angular_velocity_) / WHEEL_RADIUS;

        // Convert to RPM
        left_wheel_rpm_ = v_left * 60.0 / (2.0 * M_PI);
        right_wheel_rpm_ = v_right * 60.0 / (2.0 * M_PI);

        auto now = this->get_clock()->now();
        auto timestamp_ms = now.nanoseconds() / 1000000;  // convert to milliseconds

        // Store the shared data
        wheel_data_.left_wheel_rpm = left_wheel_rpm_;
        wheel_data_.right_wheel_rpm = right_wheel_rpm_;
        wheel_data_.target_linear_velocity = target_linear_velocity_;
        wheel_data_.target_angular_velocity = target_angular_velocity_;
        wheel_data_.timestamp_ms = timestamp_ms;

        RCLCPP_INFO(this->get_logger(),
                    "Left RPM: %f, Right RPM: %f, Linear Vel: %f, Angular Vel: %f, Timestamp: %llu",
                    left_wheel_rpm_, right_wheel_rpm_, target_linear_velocity_,
                    target_angular_velocity_, wheel_data_.timestamp_ms);
    }

    // Publisher for cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    // Subscription for cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    // Timer to trigger velocity calculation
    rclcpp::TimerBase::SharedPtr timer_;

    // Target velocities
    double target_linear_velocity_ = 0.0;
    double target_angular_velocity_ = 0.0;

    // Calculated wheel velocities
    double left_wheel_rpm_ = 0.0;
    double right_wheel_rpm_ = 0.0;

    // Shared data structure for wheel data
    WheelData wheel_data_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelVelocityNode>());
    rclcpp::shutdown();
    return 0;
}
