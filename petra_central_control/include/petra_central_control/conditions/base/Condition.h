/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Base for all BT-Conditions with ROS2
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <behaviortree_cpp_v3/condition_node.h>

#include <petra_central_control/actions/base/RosNode.h>

/**
 * Condition is the base for every node in the behavior tree 
 * which also needs acces to the ROS2 node handle. 
 * It inherits from the ConditionNode of the BT-framework.
 */
class Condition : public RosNode, public BT::ConditionNode
{
public:
    Condition(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name), BT::ConditionNode(name, config) {}

    /**
     * Method to override in each Condition. 
     * Should return SUCCESS if the condition evaluates as expected.
     */
    virtual BT::NodeStatus onCheck() = 0;

    /**
     * Method to be called when the BT-Condition is ticked. 
     * Calls onCheck() for 
     */
    BT::NodeStatus tick() override { return onCheck(); }
};