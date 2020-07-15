/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <petra_core/action/move_arm.hpp>
#include <petra_core/Point.h>

using MoveArm = petra_core::action::MoveArm;
using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

class ManipulatorDummy : public rclcpp::Node
{
public:
    ManipulatorDummy(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback_();

    rclcpp_action::Server<MoveArm>::SharedPtr move_arm_server_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_status_publisher_;

    rclcpp_action::GoalResponse handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveArm::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_(const std::shared_ptr<GoalHandleMoveArm> goal_handle);

    void handle_accepted_(const std::shared_ptr<GoalHandleMoveArm> goal_handle);
    void execute_(const std::shared_ptr<GoalHandleMoveArm> goal_handle);

    Point current_position_ = Point();

    //accuracy in mm
    float accuracy_ = 1;
    //velocity in mm/s
    float velocity_ = 100;
    
    int diagnostic_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
};