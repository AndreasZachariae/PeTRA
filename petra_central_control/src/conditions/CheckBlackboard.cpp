#include <petra_central_control/conditions/CheckBlackboard.h>

CheckBlackboard::CheckBlackboard(const std::string &name, const BT::NodeConfiguration &config) : Condition(name, config)
{
}

BT::NodeStatus CheckBlackboard::onCheck()
{
    BT::Optional<std::string> input = getInput<std::string>("input");
    BT::Optional<std::string> compare_input = getInput<std::string>("compare_to");

    if (!input.has_value())
    {
        throw BT::RuntimeError("missing required input [input]: ", input.error());
    }
    if (!compare_input.has_value())
    {
        throw BT::RuntimeError("missing required input [compare_to]: ", compare_input.error());
    }

    if (getInput<std::string>("input").value() == getInput<std::string>("compare_to").value())
    {
        log("values matched");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}