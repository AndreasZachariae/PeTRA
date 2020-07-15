/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <petra_core/Point.h>

class RobotDummy : public rclcpp::Node
{
public:
    RobotDummy();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback_();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr set_battery_publisher_;

    void goal_callback_(geometry_msgs::msg::PoseStamped::UniquePtr goal);
    void stop_callback_(const std_msgs::msg::Empty::SharedPtr msg);

    void simulate_movement_(Point goal);

    bool stop_flipflop_ = false;
    Point current_position_ = Point(0, 0);

    // accuracy in m
    float accuracy_ = 0.1;
    // velocity in m/s
    float velocity_ = 1;

    int diagnostic_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
};