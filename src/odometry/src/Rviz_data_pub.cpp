/*
 * Arjuna RViz Data Publisher - ROS 2 Conversion
 * Based on ROS 1 version with proper quaternion handling
 * Converts RViz interface messages to 2D pose format
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class RVizDataPublisher : public rclcpp::Node
{
public:
    RVizDataPublisher() : Node("rviz_click_to_2d")
    {
        // Publishers - same topics as ROS 1
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_2d", 10);
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("initial_2d", 10);
        
        // Subscribers - same topics as ROS 1  
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10,
            std::bind(&RVizDataPublisher::handle_goal, this, std::placeholders::_1));
        
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&RVizDataPublisher::handle_initial_pose, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "RViz Data Publisher (ROS 2 conversion) initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /move_base_simple/goal, /initialpose");
        RCLCPP_INFO(this->get_logger(), "Publishing to: goal_2d, initial_2d");
    }

private:
    void handle_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped rpyGoal;
        rpyGoal.header.frame_id = "map";
        rpyGoal.header.stamp = msg->header.stamp;
        rpyGoal.pose.position.x = msg->pose.position.x;
        rpyGoal.pose.position.y = msg->pose.position.y;
        rpyGoal.pose.position.z = 0.0;
        
        // FIXED: Your ROS 1 code had wrong quaternion extraction
        // Convert quaternion to yaw properly
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        rpyGoal.pose.orientation.x = 0.0;
        rpyGoal.pose.orientation.y = 0.0;
        rpyGoal.pose.orientation.z = yaw;
        rpyGoal.pose.orientation.w = 0.0;
        
        goal_pub_->publish(rpyGoal);
        RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f, yaw: %.2f)", 
                    rpyGoal.pose.position.x, rpyGoal.pose.position.y, yaw);
    }
    
    void handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped rpyPose;
        rpyPose.header.frame_id = "map";
        rpyPose.header.stamp = msg->header.stamp;
        rpyPose.pose.position.x = msg->pose.pose.position.x;
        rpyPose.pose.position.y = msg->pose.pose.position.y;
        rpyPose.pose.position.z = 0.0;
        
        // FIXED: Your ROS 1 code had wrong quaternion extraction
        // Convert quaternion to yaw properly
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        rpyPose.pose.orientation.x = 0.0;
        rpyPose.pose.orientation.y = 0.0;
        rpyPose.pose.orientation.z = yaw;
        rpyPose.pose.orientation.w = 0.0;
        
        initial_pose_pub_->publish(rpyPose);
        RCLCPP_INFO(this->get_logger(), "Initial pose set: (%.2f, %.2f, yaw: %.2f)", 
                    rpyPose.pose.position.x, rpyPose.pose.position.y, yaw);
    }
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RVizDataPublisher>());
    rclcpp::shutdown();
    return 0;
}