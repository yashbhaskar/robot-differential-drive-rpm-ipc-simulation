#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <mutex>
#include "httplib/httplib.h"

// Shared structure to store data
struct WheelData {
    double left_rpm = 0.0;
    double right_rpm = 0.0;
    double linear_vel = 0.0;
    double angular_vel = 0.0;
    uint64_t timestamp_script_a = 0;
    uint64_t timestamp_script_b = 0;
};

class WheelDataFetcherNode : public rclcpp::Node {
public:
    WheelDataFetcherNode() : Node("wheel_data_fetcher_node") {
        // Set up the HTTP server on port 8080
        server_.Get("/get_wheel_data", [this](const httplib::Request& req, httplib::Response& res) {
            this->fetch_and_send_data(res);
        });

        // Start the HTTP server in a separate thread
        std::thread([this] { server_.listen("0.0.0.0", 8080); }).detach();

        // Timer to fetch data every 100ms (10 Hz)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&WheelDataFetcherNode::fetch_data, this));
    }

private:
    void fetch_data() {
        // Here, you'd fetch data from shared memory (using some form of IPC, such as shared memory or a file).
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Simulated data from Script A
        wheel_data_.left_rpm = 52.758545;  // Example left RPM
        wheel_data_.right_rpm = 52.758545; // Example right RPM
        wheel_data_.linear_vel = 0.500000; // Example linear velocity
        wheel_data_.angular_vel = 0.0;    // Example angular velocity
        wheel_data_.timestamp_script_a = get_current_timestamp(); // Timestamp from Script A
        wheel_data_.timestamp_script_b = get_current_timestamp(); // Timestamp from Script B

        RCLCPP_INFO(this->get_logger(), "Fetched Data: Left RPM: %.2f, Right RPM: %.2f, Linear Vel: %.2f, Angular Vel: %.2f",
                    wheel_data_.left_rpm, wheel_data_.right_rpm, wheel_data_.linear_vel, wheel_data_.angular_vel);
    }

    void fetch_and_send_data(httplib::Response& res) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Convert shared data to JSON format
        std::string json_data = "{\"left_rpm\": " + std::to_string(wheel_data_.left_rpm) +
                                ", \"right_rpm\": " + std::to_string(wheel_data_.right_rpm) +
                                ", \"linear_vel\": " + std::to_string(wheel_data_.linear_vel) +
                                ", \"angular_vel\": " + std::to_string(wheel_data_.angular_vel) +
                                ", \"timestamp_script_a\": " + std::to_string(wheel_data_.timestamp_script_a) +
                                ", \"timestamp_script_b\": " + std::to_string(wheel_data_.timestamp_script_b) + "}";
        
        // Send JSON response
        res.set_content(json_data, "application/json");
    }

    uint64_t get_current_timestamp() {
        // Return current timestamp in milliseconds
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    httplib::Server server_;
    WheelData wheel_data_;
    std::mutex data_mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelDataFetcherNode>());
    rclcpp::shutdown();
    return 0;
}
