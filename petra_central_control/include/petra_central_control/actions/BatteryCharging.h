#pragma once

#include <petra_central_control/default.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <petra_central_control/actions/base/Action.h>
#include <petra_central_control/actions/types/UserDialog.h>
#include <petra_core/action/charge_battery.hpp>

using ChargeBattery = petra_core::action::ChargeBattery;
using GoalHandleChargeBattery = rclcpp_action::ClientGoalHandle<ChargeBattery>;

class BatteryCharging : public Action
{
public:
    static BT::PortsList providedPorts() { return BT::PortsList(); }

    BatteryCharging(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    void set_percentage(float goal_percentage);

private:
    rclcpp_action::Client<ChargeBattery>::SharedPtr charge_battery_client_;
    std::shared_future<std::shared_ptr<GoalHandleChargeBattery>> goal_handle_future_;

    UserDialog goal_percentage_dialog_;
    float goal_percentage_;

    void request_percentage_();
    void send_goal_();
};