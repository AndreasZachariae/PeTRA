#pragma once

#include <rclcpp_action/rclcpp_action.hpp>

#include <petra_core/action/navigate_to_pose.hpp>
#include <petra_core/Point.h>

#include <petra_central_control/Skill.h>
#include <petra_central_control/UserDialog.h>

#include <behaviortree_cpp_v3/action_node.h>

using NavigateToPose = petra_core::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class BaseMovement : public Skill, public BT::CoroActionNode
{
public:
    BaseMovement(std::shared_ptr<rclcpp::Node> node_handle);
    BaseMovement(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<rclcpp::Node> node_handle);

    BT::NodeStatus tick() override;
    void halt() override;

    void set_goal(Point goal_point);

protected:
    void init_() override;
    void spin_() override;
    void stop_() override;

private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_base_client_;
    std::shared_future<std::shared_ptr<GoalHandleNavigateToPose>> goal_handle_future_;

    UserDialog goal_point_dialog_;
    Point goal_point_ = Point();

    void request_location_();
    void send_goal_();
};