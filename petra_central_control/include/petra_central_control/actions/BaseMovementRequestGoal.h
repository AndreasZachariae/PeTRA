#pragma once

#include <petra_central_control/default.h>

#include <behaviortree_cpp_v3/action_node.h>

#include <petra_central_control/actions/base/Action.h>
#include <petra_central_control/actions/types/UserDialog.h>

class BaseMovementRequestGoal : public Action
{
public:
    static BT::PortsList providedPorts() { return { BT::OutputPort<double>("X"), BT::OutputPort<double>("Y") }; }

    BaseMovementRequestGoal(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    UserDialog goal_point_dialog_;

    bool request_sent_ = false;
    bool request_recieved_ = false;
};