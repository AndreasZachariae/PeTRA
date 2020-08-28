/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "RobotDummy"
 * Purpose : Provides a dummy for the robot base and communicates to the
 *           navigation with topics "odom" and "move_base_simple/goal".
 *           Has to be adapted to real hardware.
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <petra_core/default.h>
#include <petra_core/Point.h>

class RobotDummy : public rclcpp::Node
{
public:
    RobotDummy();

private:
    bool stop_recieved_ = false;

    Point current_position_ = Point(0, 0);

    // accuracy in m
    float accuracy_ = 0.1;
    // velocity in m/s
    float velocity_ = 1;

    unsigned char diagnostic_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr set_battery_publisher_;

    void simulate_movement_(Point goal);
};