/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Base for all BT-Actions with ROS2
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <behaviortree_cpp_v3/action_node.h>

#include <petra_central_control/actions/base/RosNode.h>

/**
 * Action is the base for every node in the behavior tree 
 * which also needs acces to the ROS2 node handle. 
 * It inherits from the StatefulActionNode of the BT-framework.
 */
class Action : public RosNode, public BT::StatefulActionNode
{
public:
    Action(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name), BT::StatefulActionNode(name, config) {}

    /**
     * Method to be called at the beginning of a BT-action. 
     * It has to return RUNNING to become asynchronous and to call onRunning afterwards.
     */
    virtual BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }
};