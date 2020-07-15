#include <petra_central_control/conditions/CheckBattery.h>

CheckBattery::CheckBattery(const std::string &name, const BT::NodeConfiguration &config) : Condition(name, config)
{
    battery_state_subscription_ = get_node_handle()->create_subscription<sensor_msgs::msg::BatteryState>("BatteryState", 10, [&](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        battery_percentage_ = msg->percentage;
    });
}

BT::NodeStatus CheckBattery::onCheck()
{
    if (battery_percentage_ <= 0.2)
    {
        log("[BatteryState] Battery less than 20%, Start 'Recharge' behavior", LogLevel::Warn);

        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}