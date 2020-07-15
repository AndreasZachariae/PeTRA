#pragma once

#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <petra_central_control/CentralControlUnit.h>

class SystemMonitor : public Skill
{
public:
    SystemMonitor(CentralControlUnit* ccu);

    void check_system();

protected:
    void spin_() override;

private:
    CentralControlUnit* ccu_ptr_ = nullptr;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_subscription_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_status_subscription_;

    bool recharge_mission_added_ = false;
};