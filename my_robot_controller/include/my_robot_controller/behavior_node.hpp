#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Enum representing the finite state machine (FSM) states of the robot
enum class RobotState {
    SEARCH_SPIN, // Rotating in place to search for the target
    SEARCH_MOVE, // Moving forward blindly to explore new areas
    TRACKING,    // Target acquired, moving towards it
    REACHED,     // Arrived at the target successfully
    RECOVERY     // Obstacle detected, performing evasion maneuvers
};

class BehaviorNode : public rclcpp::Node
{
public:
    BehaviorNode();

private:
    // Core state machine helper function
    void changeState(RobotState new_state, const std::string& message);
    
    // Sensor data callbacks
    void targetCallback(const geometry_msgs::msg::Point::SharedPtr msg); // Processes camera/vision data
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg); // Processes 2D Lidar data

    // ROS 2 Communication interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;       // Publishes velocity commands
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_; // Subscribes to target coordinates
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // Subscribes to Lidar scans
    
    // State machine tracking variables
    RobotState current_state_;
    rclcpp::Time last_state_change_time_; 
    
    // Obstacle avoidance and spatial memory variables
    double front_distance_; // Distance to the nearest obstacle in the forward view (meters)
    int min_lidar_index_;   // Index of the Lidar ray pointing to the closest obstacle
    double last_spin_dir_;  // Spatial memory: direction to spin based on last known target position

};