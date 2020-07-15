#pragma once

#include <petra_central_control/default.h>

#include <sensor_msgs/msg/battery_state.hpp>

#include <petra_central_control/conditions/base/Condition.h>

class CheckBattery : public Condition
{
public:
    static BT::PortsList providedPorts() { return BT::PortsList(); }

    CheckBattery(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onCheck() override;

private:
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_subscription_;

    float battery_percentage_ = 1;
};
