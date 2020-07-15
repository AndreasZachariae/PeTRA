#pragma once

#include <petra_central_control/default.h>

#include <petra_central_control/conditions/base/Condition.h>

class CheckBlackboard : public Condition
{
public:
    static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("input"), BT::InputPort<std::string>("compare_to") }; }

    CheckBlackboard(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onCheck() override;
};
