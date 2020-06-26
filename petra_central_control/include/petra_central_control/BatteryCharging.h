#pragma once

#include <rclcpp_action/rclcpp_action.hpp>

#include <petra_core/action/charge_battery.hpp>

#include <petra_central_control/default.h>
#include <petra_central_control/Skill.h>
#include <petra_central_control/UserDialog.h>

using ChargeBattery = petra_core::action::ChargeBattery;
using GoalHandleChargeBattery = rclcpp_action::ClientGoalHandle<ChargeBattery>;

class BatteryCharging : public Skill
{
public:
    BatteryCharging(std::shared_ptr<rclcpp::Node> node_handle);

protected:
    void spin_() override;
    void stop_() override;

private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    rclcpp_action::Client<ChargeBattery>::SharedPtr charge_battery_client_;
    std::shared_future<std::shared_ptr<GoalHandleChargeBattery>> goal_handle_future_;

    UserDialog goal_percentage_dialog_;
    float goal_percentage_;

    void request_percentage_();
    void send_goal_();
};