/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "BatteryDummy"
 * Purpose : Provides the ROS2-Action server "ChargeBattery" 
 *           and simulates battery charging.
 *           Has to be adapted to real hardware.
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/float32.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <petra_core/default.h>
#include <petra_core/action/charge_battery.hpp>

using ChargeBattery = petra_core::action::ChargeBattery;
using GoalHandleChargeBattery = rclcpp_action::ServerGoalHandle<ChargeBattery>;

class BatteryDummy : public rclcpp::Node
{
public:
    BatteryDummy();

private:
    float battery_percentage_ = 0.5;

    unsigned char diagnostic_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<ChargeBattery>::SharedPtr charge_battery_server_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_status_publisher_;

    //only for testing
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_battery_subscription_;

    rclcpp_action::GoalResponse handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ChargeBattery::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle);
    void handle_accepted_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle);
};