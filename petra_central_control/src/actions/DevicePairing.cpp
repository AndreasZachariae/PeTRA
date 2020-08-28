#include <petra_central_control/actions/DevicePairing.h>

void DevicePairing::onSend(PairDevice::Goal &goal)
{
    // Check if Optional is valid. If not, throw its error
    BT::Optional<int> mode_input = getInput<int>("pairing_mode");
    BT::Optional<int> device_input = getInput<int>("device_type");

    if (mode_input.has_value())
    {
        if ((mode_input.value() == PairDevice::Goal::PAIR) || (mode_input.value() == PairDevice::Goal::UNPAIR))
        {
            goal.pairing_mode = mode_input.value();
        }
        else
        {
            throw BT::RuntimeError("wrong input for pairing_mode, use constants from .action file", mode_input.error());
        }
    }
    else
    {
        throw BT::RuntimeError("missing required input [pairing_mode]: ", mode_input.error());
    }

    if (device_input.has_value())
    {
        if ((device_input.value() == PairDevice::Goal::WHEELCHAIR) || (device_input.value() == PairDevice::Goal::ROLLATOR) || (device_input.value() == PairDevice::Goal::HOSPITALBED) || (device_input.value() == PairDevice::Goal::CHARGINGSTATION))
        {
            goal.device_type = device_input.value();
        }
        else
        {
            throw BT::RuntimeError("wrong input for device_type, use constants from .action file", device_input.error());
        }
    }
    else
    {
        throw BT::RuntimeError("missing required input [device_type]: ", device_input.error());
    }

    goal.header.stamp = get_node_handle()->now();
}

void DevicePairing::onFeedback(const std::shared_ptr<const PairDevice::Feedback> feedback)
{
    log("Pairing progress: " + ftos(feedback->progress * 100) + " %");
}

void DevicePairing::onResult(const rclcpp_action::ClientGoalHandle<PairDevice>::WrappedResult &, const PairDevice::Goal &goal)
{
    log("Device pairing succeeded!, Total time: " + std::to_string((int)(get_node_handle()->now().seconds() - goal.header.stamp.sec)) + "s");
}