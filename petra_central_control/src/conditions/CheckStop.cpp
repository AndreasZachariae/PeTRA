#include <petra_central_control/conditions/CheckStop.h>

CheckStop::CheckStop(const std::string &name, const BT::NodeConfiguration &config) : Condition(name, config)
{
    stop_subscription_ = get_node_handle()->create_subscription<std_msgs::msg::Empty>("Stop", 10, [&](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        stop_recieved_ = true;
    });
}

BT::NodeStatus CheckStop::onCheck()
{
    if (stop_recieved_)
    {
        log("Stop failed");

        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}