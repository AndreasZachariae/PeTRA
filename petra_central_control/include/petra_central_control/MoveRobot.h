#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/empty.hpp>
#include <petra_core/action/navigate_to_pose.hpp>
#include <petra_core/Point.h>

#include <petra_central_control/default.h>
#include <petra_central_control/Skill.h>
#include <petra_central_control/Dialogue.h>

using NavigateToPose = petra_core::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class MoveRobot : public Skill
{
public:
    MoveRobot(std::shared_ptr<rclcpp::Node> node_handle);

protected:
    void spin_() override;
    void stop_() override;

private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    std::shared_ptr<rclcpp::Node> ccu_node_handle_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    std::shared_future<std::shared_ptr<GoalHandleNavigateToPose>> goal_handle_future_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;

    Dialogue goal_point_dialogue_;
    Point* goal_point_ = nullptr;

    void request_data_();
    void send_goal_();
};