/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <petra_core/action/navigate_to_pose.hpp>
#include <petra_core/Point.h>

using NavigateToPose = petra_core::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

class Navigation2Dummy : public rclcpp::Node
{
public:
    Navigation2Dummy(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_to_pose_server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

    rclcpp_action::GoalResponse handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const NavigateToPose::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

    void handle_accepted_(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
    void execute_(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

    void odometry_callback(nav_msgs::msg::Odometry::UniquePtr goal);

    Point current_position_ = Point();
    float accuracy_ = 0.1;
};