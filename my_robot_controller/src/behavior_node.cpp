#include "my_robot_controller/behavior_node.hpp"
#include <limits>

BehaviorNode::BehaviorNode() : Node("behavior_node"), current_state_(RobotState::SEARCH_SPIN)
{
    // Declare tunable ROS 2 parameters for dynamic configuration
    this->declare_parameter<double>("kp_angular", 0.003);
    this->declare_parameter<double>("kp_linear", 0.000015);
    this->declare_parameter<double>("max_speed", 1.5);
    this->declare_parameter<double>("spin_time", 5.0);
    this->declare_parameter<double>("move_time", 3.0);
    this->declare_parameter<double>("target_area", 60000.0); 
    this->declare_parameter<double>("safe_distance", 0.6); 

    // Initialize sensor variables
    front_distance_ = std::numeric_limits<double>::infinity();
    min_lidar_index_ = 180; // 180 is directly forward
    
    // Default spatial memory: spin left (counter-clockwise)
    last_spin_dir_ = 0.5;

    // Setup publisher for velocity commands
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Setup subscriber for target detection data (camera)
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/target_info", 10,
        std::bind(&BehaviorNode::targetCallback, this, std::placeholders::_1));

    // Setup subscriber for obstacle detection (lidar)
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&BehaviorNode::scanCallback, this, std::placeholders::_1));

    last_state_change_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Behavior Node Started!");
}

// Helper function to safely transition between states and log the change
void BehaviorNode::changeState(RobotState new_state, const std::string& message) 
{
    current_state_ = new_state;
    last_state_change_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
}

void BehaviorNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    double min_dist = std::numeric_limits<double>::infinity();
    int best_index = 180; 

    // Scan a limited forward-facing field of view (indices 135 to 225)
    for (int i = 135; i <= 225; ++i) {
        // Ignore noise closer than 0.15m (often part of the robot's own chassis)
        if (msg->ranges[i] < min_dist && msg->ranges[i] > 0.15) { 
            min_dist = msg->ranges[i];
            best_index = i; 
        }
    }
    
    // Update global obstacle data for the state machine
    front_distance_ = min_dist;
    min_lidar_index_ = best_index;
}

void BehaviorNode::targetCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    geometry_msgs::msg::Twist cmd_msg;
    
    // Determine if target is valid based on bounding box area (msg->z represents area)
    bool cube_visible = (msg->z > 250); 
    double elapsed_sec = (this->now() - last_state_change_time_).seconds();

    double target_area = this->get_parameter("target_area").as_double();
    double safe_distance = this->get_parameter("safe_distance").as_double();

    // SPATIAL MEMORY: While the target is visible, remember which side of the screen it's on.
    // If we lose it, we will spin in this direction to reacquire it faster.
    if (cube_visible) {
        if (msg->x < 320) {
            last_spin_dir_ = 0.5;  // Target is on the left, remember to spin left
        } else {
            last_spin_dir_ = -0.5; // Target is on the right, remember to spin right
        }
    }

    // MAIN STATE MACHINE
    switch (current_state_) {
        case RobotState::SEARCH_SPIN:
            if (cube_visible) {
                changeState(RobotState::TRACKING, "Target spotted! TRACKING.");
            } else if (elapsed_sec > this->get_parameter("spin_time").as_double()) {
                changeState(RobotState::SEARCH_MOVE, "Nothing here. Moving forward...");
            } else {
                // Apply spatial memory: spin in the direction the target was last seen
                cmd_msg.angular.z = last_spin_dir_; 
            }
            break;

        case RobotState::SEARCH_MOVE:
            if (cube_visible) {
                changeState(RobotState::TRACKING, "Target spotted! TRACKING.");
            } else if (front_distance_ < safe_distance) {
                changeState(RobotState::SEARCH_SPIN, "Obstacle ahead! Turning away...");
            } else if (elapsed_sec > this->get_parameter("move_time").as_double()) {
                changeState(RobotState::SEARCH_SPIN, "Stop. Scanning new area...");
            } else {
                cmd_msg.linear.x = 0.4; // Move forward blindly to explore
            }
            break;

        case RobotState::TRACKING:
            if (!cube_visible) {
                changeState(RobotState::SEARCH_SPIN, "Lost it! Searching...");
            } else {
                if (front_distance_ < 0.35) {
                    changeState(RobotState::RECOVERY, "Too close to obstacle! Evading...");
                } else {
                    // Proportional control to center the target and adjust speed
                    double error_area = target_area - msg->z;
                    int error_x = 320 - msg->x;

                    // Steer based on horizontal offset from image center (320px)
                    cmd_msg.angular.z = error_x * this->get_parameter("kp_angular").as_double();

                    // Accelerate based on distance (area difference), up to max_speed
                    if (error_area > 0) {
                        cmd_msg.linear.x = error_area * this->get_parameter("kp_linear").as_double();
                        double max_sp = this->get_parameter("max_speed").as_double();
                        if (cmd_msg.linear.x > max_sp) cmd_msg.linear.x = max_sp;
                    } else {
                        changeState(RobotState::REACHED, "Arrived at target.");
                    }
                }
            }
            break;

        case RobotState::RECOVERY:
            cmd_msg.linear.x = -0.2; // Move in reverse
            
            // LIDAR DEADBAND LOGIC: Prevents steering oscillation (jittering)
            // when the obstacle is positioned directly in front of the robot.
            if (min_lidar_index_ > 190) {
                // Obstacle is clearly to the left, steer right
                cmd_msg.angular.z = -0.6; 
            } else if (min_lidar_index_ < 170) {
                // Obstacle is clearly to the right, steer left
                cmd_msg.angular.z = 0.6;  
            } else {
                // Obstacle is in the center deadband (170-190), reverse straight
                cmd_msg.angular.z = 0.0;
            }
            
            // Exit recovery if clear of obstacle or if stuck in recovery too long (timeout)
            if (front_distance_ > 0.50 || elapsed_sec > 2.0) {
                changeState(RobotState::TRACKING, "Obstacle cleared! Resuming tracking.");
            }
            break;

        case RobotState::REACHED:
            // Halt the robot upon reaching the target
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
            
            // Resume tracking if the target disappears or moves significantly further away
            if (!cube_visible || (target_area - msg->z > 5000)) {
                changeState(RobotState::TRACKING, "Target moved! Following...");
            }
            break;
    }

    cmd_pub_->publish(cmd_msg);
}

int main(int argc, char* argv[]) 
{
    // Standard ROS 2 node execution cycle
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorNode>());
    rclcpp::shutdown();
    return 0;
}