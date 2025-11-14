/*
 * Arjuna EKF Odometry Publisher - WITH TF CONTROL
 * Use publish_tf parameter to control TF broadcasting
 * Set to false when using robot_localization
 */

#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class EKFOdomPublisher : public rclcpp::Node
{
public:
    EKFOdomPublisher() : Node("ekf_odom_pub")
    {
        // Declare parameter for TF broadcasting control
        this->declare_parameter("publish_tf", true);
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        
        // Robot parameters
        WHEEL_RADIUS = 0.04;
        WHEEL_BASE = 0.28;
        TICKS_PER_METER = 20475.0;
        PI = 3.141592653589793;
        
        robot_x = 0.0;
        robot_y = 0.0;
        robot_theta = 0.0;
        
        lastCountL = 0;
        lastCountR = 0;
        distanceLeft = 0.0;
        distanceRight = 0.0;
        first_left_received = true;
        first_right_received = true;
        
        linear_velocity = 0.0;
        angular_velocity = 0.0;
        
        last_time = this->now();
        
        // Publishers
        odom_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_euler", 100);
        odom_data_pub_quat_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_quat", 100);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
        
        // Subscribers
        right_ticks_sub_ = this->create_subscription<std_msgs::msg::Int64>(
            "right_ticks", 100, 
            std::bind(&EKFOdomPublisher::calc_right, this, std::placeholders::_1));
        left_ticks_sub_ = this->create_subscription<std_msgs::msg::Int64>(
            "left_ticks", 100, 
            std::bind(&EKFOdomPublisher::calc_left, this, std::placeholders::_1));
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "initial_2d", 1, 
            std::bind(&EKFOdomPublisher::set_initial_2d, this, std::placeholders::_1));
        
        // TF broadcaster (only if enabled)
        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&EKFOdomPublisher::update_odom, this));
        
        RCLCPP_INFO(this->get_logger(), "=========================================");
        RCLCPP_INFO(this->get_logger(), "  Arjuna Wheel Odometry Node");
        RCLCPP_INFO(this->get_logger(), "=========================================");
        RCLCPP_INFO(this->get_logger(), "Wheel base: %.3f m", WHEEL_BASE);
        RCLCPP_INFO(this->get_logger(), "Wheel radius: %.3f m", WHEEL_RADIUS);
        RCLCPP_INFO(this->get_logger(), "TF broadcasting: %s", publish_tf_ ? "ENABLED" : "DISABLED");
        if (!publish_tf_) {
            RCLCPP_INFO(this->get_logger(), "TF will be published by robot_localization");
        }
        RCLCPP_INFO(this->get_logger(), "=========================================");
    }

private:
    
    void set_initial_2d(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        robot_x = msg->pose.position.x;
        robot_y = msg->pose.position.y;
        robot_theta = msg->pose.orientation.z;
        RCLCPP_INFO(this->get_logger(), "Initial pose: x=%.3f, y=%.3f, theta=%.3f", 
                    robot_x, robot_y, robot_theta);
    }
    
    void calc_left(const std_msgs::msg::Int64::SharedPtr msg)
    {
        if (first_left_received) {
            first_left_received = false;
            lastCountL = msg->data;
            return;
        }
        
        int64_t leftTicks = msg->data - lastCountL;
        
        if (leftTicks > 10000) {
            leftTicks = 0 - (65535 - leftTicks);
        } else if (leftTicks < -10000) {
            leftTicks = 65535 - leftTicks;
        }
        
        distanceLeft = static_cast<double>(leftTicks) / TICKS_PER_METER;
        lastCountL = msg->data;
    }
    
    void calc_right(const std_msgs::msg::Int64::SharedPtr msg)
    {
        if (first_right_received) {
            first_right_received = false;
            lastCountR = msg->data;
            return;
        }
        
        int64_t rightTicks = msg->data - lastCountR;
        
        if (rightTicks > 10000) {
            rightTicks = 0 - (65535 - rightTicks);
        } else if (rightTicks < -10000) {
            rightTicks = 65535 - rightTicks;
        }
        
        distanceRight = static_cast<double>(rightTicks) / TICKS_PER_METER;
        lastCountR = msg->data;
    }
    
    void update_odom()
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time).seconds();
        
        double distance_center = (distanceRight + distanceLeft) / 2.0;
        double delta_theta = (distanceRight - distanceLeft) / WHEEL_BASE;
        
        double delta_x = distance_center * cos(robot_theta + delta_theta / 2.0);
        double delta_y = distance_center * sin(robot_theta + delta_theta / 2.0);
        
        robot_x += delta_x;
        robot_y += delta_y;
        robot_theta += delta_theta;
        
        while (robot_theta > PI) robot_theta -= 2.0 * PI;
        while (robot_theta < -PI) robot_theta += 2.0 * PI;
        
        if (dt > 0.001) {
            linear_velocity = distance_center / dt;
            angular_velocity = delta_theta / dt;
        } else {
            linear_velocity = 0.0;
            angular_velocity = 0.0;
        }
        
        publish_odometry_messages(current_time);
        
        // Only broadcast TF if enabled (disabled when using robot_localization)
        if (publish_tf_) {
            broadcast_tf(current_time);
        }
        
        distanceLeft = 0.0;
        distanceRight = 0.0;
        last_time = current_time;
        
        static int debug_counter = 0;
        if (++debug_counter >= 90) {
            debug_counter = 0;
            RCLCPP_INFO(this->get_logger(), 
                "Pose: x=%.3f, y=%.3f, θ=%.3f | Vel: lin=%.3f, ang=%.3f", 
                robot_x, robot_y, robot_theta, linear_velocity, angular_velocity);
        }
    }
    
    void publish_odometry_messages(const rclcpp::Time& stamp)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, robot_theta);
        
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        odom_msg.pose.pose.position.x = robot_x;
        odom_msg.pose.pose.position.y = robot_y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        odom_msg.twist.twist.linear.x = linear_velocity;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = angular_velocity;
        
        odom_msg.pose.covariance[0] = 0.01;
        odom_msg.pose.covariance[7] = 0.01;
        odom_msg.pose.covariance[14] = 0.01;
        odom_msg.pose.covariance[21] = 0.1;
        odom_msg.pose.covariance[28] = 0.1;
        odom_msg.pose.covariance[35] = 0.1;
        
        odom_msg.twist.covariance[0] = 0.01;
        odom_msg.twist.covariance[7] = 0.01;
        odom_msg.twist.covariance[14] = 0.01;
        odom_msg.twist.covariance[21] = 0.1;
        odom_msg.twist.covariance[28] = 0.1;
        odom_msg.twist.covariance[35] = 0.1;
        
        odom_pub_->publish(odom_msg);
        odom_data_pub_quat_->publish(odom_msg);
        
        nav_msgs::msg::Odometry odom_euler = odom_msg;
        odom_euler.pose.pose.orientation.z = robot_theta;
        odom_data_pub_->publish(odom_euler);
    }
    
    void broadcast_tf(const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        
        transform.transform.translation.x = robot_x;
        transform.transform.translation.y = robot_y;
        transform.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, robot_theta);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
    }
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_ticks_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_ticks_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_METER, PI;
    double robot_x, robot_y, robot_theta;
    double linear_velocity, angular_velocity;
    int64_t lastCountL, lastCountR;
    double distanceLeft, distanceRight;
    bool first_left_received, first_right_received;
    bool publish_tf_;  // NEW: Control TF publishing
    rclcpp::Time last_time;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}