#include <petra_central_control/conditions/CheckBlackboard.h>

CheckBlackboard::CheckBlackboard(const std::string &name, const BT::NodeConfiguration &config) : Condition(name, config)
{
}

BT::NodeStatus CheckBlackboard::onCheck()
{
    if (getInput<std::string>("input").value() == getInput<std::string>("compare_to").value())
    {
        log("values matched");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}