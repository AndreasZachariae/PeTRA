/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PairingModuleDummy"
 * Purpose : Provides the ROS2-Action server "PairDevice" 
 *           and simulates device pairing.
 *           Has to be adapted to real hardware.
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <petra_core/default.h>
#include <petra_core/action/pair_device.hpp>

using PairDevice = petra_core::action::PairDevice;
using GoalHandlePairDevice = rclcpp_action::ServerGoalHandle<PairDevice>;

class PairingModuleDummy : public rclcpp::Node
{
public:
    PairingModuleDummy();

private:
    bool paired_ = false;

    unsigned char device_;
    unsigned char diagnostic_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<PairDevice>::SharedPtr pair_device_server_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_status_publisher_;

    rclcpp_action::GoalResponse handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PairDevice::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_(const std::shared_ptr<GoalHandlePairDevice> goal_handle);
    void handle_accepted_(const std::shared_ptr<GoalHandlePairDevice> goal_handle);
};