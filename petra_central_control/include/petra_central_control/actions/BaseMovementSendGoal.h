#pragma once

#include <petra_central_control/default.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include <petra_central_control/actions/base/Action.h>
#include <petra_core/action/navigate_to_pose.hpp>
#include <petra_core/Point.h>

using NavigateToPose = petra_core::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class BaseMovementSendGoal : public Action
{
public:
    static BT::PortsList providedPorts() { return { BT::InputPort<double>("X"), BT::InputPort<double>("Y") }; }

    BaseMovementSendGoal(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    std::shared_future<std::shared_ptr<GoalHandleNavigateToPose>> goal_handle_future_;

    Point goal_point_ = Point();

    void send_goal_();
};