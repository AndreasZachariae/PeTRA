/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <petra_core/action/pair_device.hpp>

using PairDevice = petra_core::action::PairDevice;
using GoalHandlePairDevice = rclcpp_action::ServerGoalHandle<PairDevice>;

class PairingModuleDummy : public rclcpp::Node
{
public:
    PairingModuleDummy(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    rclcpp_action::Server<PairDevice>::SharedPtr pair_device_server_;

    rclcpp_action::GoalResponse handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PairDevice::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_(const std::shared_ptr<GoalHandlePairDevice> goal_handle);

    void handle_accepted_(const std::shared_ptr<GoalHandlePairDevice> goal_handle);
    void execute_(const std::shared_ptr<GoalHandlePairDevice> goal_handle);

    bool paired_ = false;
    int device_;
};