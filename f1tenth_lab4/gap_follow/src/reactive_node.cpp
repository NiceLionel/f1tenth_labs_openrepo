#include "rclcpp/rclcpp.hpp"
#include <string>
#include <iostream>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <limits>
#define PI 3.1415926

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        // RCLCPP_WARN(this->get_logger(), "The node is up!");
        // std::cout << "The node is up!" << std::endl;
        pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        // sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        // RCLCPP_INFO(this->get_logger(), "angle_min: %f", scan_msg->angle_min);
        // Preprocess the LiDAR scan and get a smaller range
        ranges = std::vector<float>(std::begin(scan_msg->ranges), std::end(scan_msg->ranges));
        const double left_beam = - 69 / 180.0 * PI;
        const double right_beam = 69 / 180.0 * PI;
        const int left_idx = (int)(std::floor((left_beam - scan_msg->angle_min) / scan_msg->angle_increment));
        const int right_idx = (int)(std::ceil((right_beam - scan_msg->angle_min) / scan_msg->angle_increment));
        RCLCPP_INFO(this->get_logger(), "length: %d", ranges.size());
        RCLCPP_INFO(this->get_logger(), "left index: %d", left_idx);
        RCLCPP_INFO(this->get_logger(), "right: %d", right_idx);
        // RCLCPP_INFO(this->get_logger(), "The node is up!");
        // std::cout << "The node is up!" << std::endl;
        // Use sliding window to calculate the mean
        int window_size = 2;
        std::vector<float> temp_range = ranges;
        for(int i = left_idx; i < right_idx; i++){
            float temp = 0;
            for(int j = -window_size; j <= window_size; j++){
                temp += ranges[i+j];
            }
            temp_range[i] = temp / (2 * window_size + 1);
        }
        ranges = temp_range;

        for(int i = left_idx; i <= right_idx; i++){
            // if (std::isinf(scan_msg->ranges[i]) || std::isnan(scan_msg->ranges[i])) {
            //     ranges[i] = 0.0;
            // } else if (scan_msg->ranges[i] > scan_mag->range_max) {
            //     ranges[i] = scan_msg->range_max;
            // }
            // if (scan_msg->ranges[i] > scan_msg->range_max){
            //     ranges[i] = scan_msg->range_max;
            // }
            // if (ranges[i] > 5.0){
            //     ranges[i] = 5.0;
            // }
            if (std::isinf(scan_msg->ranges[i]) || std::isnan(scan_msg->ranges[i])) {
                ranges[i] = 0.0;
            } else if (scan_msg->ranges[i] > scan_msg->range_max) {
                ranges[i] = scan_msg->range_max;
            }
        }

        /// TODO:
        // Find closest point to LiDAR
        int closest_idx = left_idx;
        float closest_dist = scan_msg->range_max;
        for(int i = left_idx; i <= right_idx; i++){
            if(ranges[i] <= closest_dist){
                closest_dist = ranges[i];
                closest_idx = i;
            }
        }
        RCLCPP_INFO(this->get_logger(), "closest idx: %d", closest_idx);
        RCLCPP_INFO(this->get_logger(), "closest dist: %f", closest_dist);
        // Eliminate all points inside 'bubble' (set them to zero) 
        int bub_rad = 185;
        for(int i = closest_idx - bub_rad; i <= closest_idx + bub_rad; i++){
            ranges[i] = 0;
        }
        // Find max length gap 
        int max_len = 0;
        int cur_len = 0;
        int start = left_idx;
        int stop = left_idx;
        int cur_start = left_idx;
        for(int i = left_idx; i <= right_idx; i++){
            if(ranges[i] != 0){
                cur_len++;
                if(cur_len > max_len){
                    max_len = cur_len;
                    start = cur_start;
                    stop = i;
                }
            }
            else{
                cur_start = i + 1;
                cur_len = 0;
            }
        }
        // Find the best point in the gap 
        RCLCPP_INFO(this->get_logger(), "start: %d", start);
        RCLCPP_INFO(this->get_logger(), "stop: %d", stop);
        float best_dist = 0.0;
        for(int i = start; i <= stop; i++){
            // RCLCPP_INFO(this->get_logger(), "best_dist: %d", best_dist);
            if(ranges[i] > best_dist){
                best_dist = ranges[i];
                angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            }
            else if(ranges[i] == best_dist){
                if (std::abs(scan_msg->angle_min + i * scan_msg->angle_increment) < std::abs(angle)) {
                    angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                }
            }
        }

        float temp_angle = angle / PI * 180;
        RCLCPP_INFO(this->get_logger(), "best_dist: %f", best_dist);
        RCLCPP_INFO(this->get_logger(), "angle: %f", angle);
        RCLCPP_INFO(this->get_logger(), "angle in degree: %f", temp_angle);
        // Publish Drive message
        ackermann_msgs::msg::AckermannDriveStamped action;
        action.drive.steering_angle = angle;
        if(std::abs(angle) <= 10.0 / 180.0 * PI){
            action.drive.speed = 0.9;
        }
        else if(std::abs(angle) >= 20.0 / 180 * PI){
            action.drive.speed = 0.5;
        }
        else{
            action.drive.speed = 0.7;
        }
        RCLCPP_INFO(this->get_logger(), "Speed: %f", action.drive.speed);
        pub_->publish(action);
        RCLCPP_INFO(this->get_logger(), "///////////////////////");
    }



private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    std::vector<float> ranges;
    float angle;

};

int main(int argc, char ** argv) {
    std::cout << "The node is up! 1" << std::endl;
    // RCLCPP_INFO(this->get_logger(), "Test");
    rclcpp::init(argc, argv);
    std::cout << "The node is up! 2" << std::endl;
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    std::cout << "The node is up! 3" << std::endl;
    rclcpp::shutdown();
    return 0;
}