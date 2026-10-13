#include "my_robot_controller/vision_node.hpp"

VisionTrackerNode::VisionTrackerNode() : Node("vision_tracker_node")
{
    // Subscriber for the raw camera feed from Gazebo/real camera
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&VisionTrackerNode::imageCallback, this, std::placeholders::_1));

    // Publisher for the processed image stream (useful for debugging in RViz/rqt)
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision_tracker/image_processed", 10);
    
    // Publisher for target coordinates. This acts as the "Eyes" sending data to the "Brain" (Behavior Node)
    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/target_info", 10);

    RCLCPP_INFO(this->get_logger(), "Perception Node Started! Looking for red cubes...");
}

void VisionTrackerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat frame;
    
    // Convert ROS 2 Image message to OpenCV Matrix format
    try { 
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image; 
    } catch (cv_bridge::Exception& e) { 
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return; 
    }

    cv::Mat hsv, mask;
    // Convert BGR image to HSV color space for robust color detection
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // Threshold the HSV image to extract only the red color
    // Note: This range covers the lower red spectrum in OpenCV (Hue 0-10)
    cv::inRange(hsv, cv::Scalar(0, 150, 100), cv::Scalar(10, 255, 255), mask);

    // Calculate image moments to find the center of mass of the detected red blob
    cv::Moments m = cv::moments(mask, true);
    
    // Prepare the message to send target data
    // We use a Point message: x and y for screen coordinates, and cleverly use z for the blob's area
    geometry_msgs::msg::Point target_msg;

    // Filter out small noise (area must be > 250 pixels)
    if (m.m00 > 250) { 
        target_msg.x = m.m10 / m.m00; // Calculate Center X
        target_msg.y = m.m01 / m.m00; // Calculate Center Y
        target_msg.z = m.m00;         // Store the blob's area in Z (correlates to distance)

        // Draw a green circle at the calculated center for visual debugging
        cv::circle(frame, cv::Point(target_msg.x, target_msg.y), 10, cv::Scalar(0, 255, 0), -1);
    } else {
        // Target not found or too small, reset values
        target_msg.x = 0; 
        target_msg.y = 0; 
        target_msg.z = 0; 
    }

    // Publish the extracted coordinates to the behavior node
    target_pub_->publish(target_msg); 

    // Draw a vertical blue line exactly in the middle of the screen (320px) 
    // to visualize the center reference point for the robot's steering
    cv::line(frame, cv::Point(320, 0), cv::Point(320, 480), cv::Scalar(255, 0, 0), 2);
    
    // Convert the annotated OpenCV frame back to a ROS 2 Image message and publish it
    sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    image_pub_->publish(*out_msg);
}

int main(int argc, char* argv[]) {
    // Standard ROS 2 node execution cycle
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionTrackerNode>());
    rclcpp::shutdown();
    return 0;
}