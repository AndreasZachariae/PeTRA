#pragma once

#include <petra_central_control/default.h>

#include <behaviortree_cpp_v3/action_node.h>

#include <petra_central_control/actions/base/RosNode.h>

class Action : public RosNode, public BT::StatefulActionNode
{
public:
    Action(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name), BT::StatefulActionNode(name, config) {}

    virtual BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }
};