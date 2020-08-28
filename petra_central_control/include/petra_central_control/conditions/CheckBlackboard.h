/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Checks if the given value macthes the blackboard entry
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <petra_central_control/conditions/base/Condition.h>

class CheckBlackboard : public Condition
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("input"), BT::InputPort<std::string>("compare_to")}; }

    CheckBlackboard(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onCheck() override;
};
