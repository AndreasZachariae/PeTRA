/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "ManipulatorDummy"
 * Purpose : Provides the ROS2-Action server "MoveArm" 
 *           and simulates arm movement.
 *           Has to be adapted to real hardware.
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <petra_core/default.h>
#include <petra_core/action/move_arm.hpp>
#include <petra_core/Point.h>

using MoveArm = petra_core::action::MoveArm;
using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

class ManipulatorDummy : public rclcpp::Node
{
public:
    ManipulatorDummy();

private:
    Point current_position_ = Point(0, 0);

    //accuracy in mm
    float accuracy_ = 1;
    //velocity in mm/s
    float velocity_ = 100;

    unsigned char diagnostic_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<MoveArm>::SharedPtr move_arm_server_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_status_publisher_;

    rclcpp_action::GoalResponse handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveArm::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_(const std::shared_ptr<GoalHandleMoveArm> goal_handle);
    void handle_accepted_(const std::shared_ptr<GoalHandleMoveArm> goal_handle);
};