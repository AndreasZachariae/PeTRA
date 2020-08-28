#include <petra_central_control/actions/BatteryCharging.h>

void BatteryCharging::onSend(ChargeBattery::Goal &goal)
{
    // Check if Optional is valid. If not, throw its error
    BT::Optional<float> percentage_input = getInput<float>("goal_percentage");

    if (!percentage_input.has_value())
    {
        throw BT::RuntimeError("missing required input [goal_percentage]: ", percentage_input.error());
    }

    goal.goal_percentage = percentage_input.value();
    goal.header.stamp = get_node_handle()->now();

    log("Goal: " + ftos(goal.goal_percentage * 100) + "%");
}

void BatteryCharging::onFeedback(const std::shared_ptr<const ChargeBattery::Feedback> feedback)
{
    log("Battery level: " + ftos(feedback->current_state.percentage * 100) + "%");
}

void BatteryCharging::onResult(const rclcpp_action::ClientGoalHandle<ChargeBattery>::WrappedResult &, const ChargeBattery::Goal &goal)
{
    log("Finished! Total time: " + std::to_string((int)(get_node_handle()->now().seconds() - goal.header.stamp.sec)) + "s");
}