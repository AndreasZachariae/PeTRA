#pragma once

#include <petra_central_control/default.h>

#include <behaviortree_cpp_v3/condition_node.h>

#include <petra_central_control/actions/base/RosNode.h>

class Condition : public RosNode, public BT::ConditionNode
{
public:
    Condition(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name), BT::ConditionNode(name, config) {}

    virtual BT::NodeStatus onCheck() = 0;

    BT::NodeStatus tick() override { return onCheck(); }
};