#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp> 


class VisionTrackerNode : public rclcpp::Node
{
public:
    // Constructor: Initializes ROS 2 publishers and subscribers
    VisionTrackerNode();

private:
    // Callback function triggered every time a new image frame is received from the camera
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // ROS 2 Communication interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // Subscribes to the raw camera feed
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_; // Publishes the calculated target coordinates (x, y, area)
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;    // Publishes the annotated image for visual debugging in RViz/rqt
};