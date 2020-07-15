#pragma once

#include <rclcpp_action/rclcpp_action.hpp>

#include <petra_core/action/move_arm.hpp>
#include <petra_core/Point.h>

#include <petra_central_control/Skill.h>
#include <petra_central_control/UserDialog.h>

using MoveArm = petra_core::action::MoveArm;
using GoalHandleMoveArm = rclcpp_action::ClientGoalHandle<MoveArm>;

class ArmMovement : public Skill
{
public:
    ArmMovement(std::shared_ptr<rclcpp::Node> node_handle);

protected:
    void spin_() override;
    void stop_() override;

    void set_coordinates(Point goal_point, float gripper_position);

private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    rclcpp_action::Client<MoveArm>::SharedPtr move_arm_client_;
    std::shared_future<std::shared_ptr<GoalHandleMoveArm>> goal_handle_future_;

    UserDialog goal_point_dialog_;
    Point goal_point_ = Point();
    float gripper_position_ = 0;

    void request_coordinates_();
    void send_goal_();
};