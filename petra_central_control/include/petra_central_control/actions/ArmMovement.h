#pragma once

#include <petra_central_control/default.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <petra_central_control/actions/base/Action.h>
#include <petra_central_control/actions/types/UserDialog.h>
#include <petra_core/action/move_arm.hpp>
#include <petra_core/Point.h>

using MoveArm = petra_core::action::MoveArm;
using GoalHandleMoveArm = rclcpp_action::ClientGoalHandle<MoveArm>;

class ArmMovement : public Action
{
public:
    static BT::PortsList providedPorts() { return BT::PortsList(); }

    ArmMovement(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    void set_coordinates(Point goal_point, float gripper_position);

private:
    rclcpp_action::Client<MoveArm>::SharedPtr move_arm_client_;
    std::shared_future<std::shared_ptr<GoalHandleMoveArm>> goal_handle_future_;

    UserDialog goal_point_dialog_;
    Point goal_point_ = Point();
    float gripper_position_ = 0;

    void request_coordinates_();
    void send_goal_();
};