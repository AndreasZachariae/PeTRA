/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Provides the ROS2-Action client "MoveArm"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <petra_central_control/actions/base/RosAction.h>
#include <petra_central_control/actions/base/converts.h>
#include <petra_core/action/move_arm.hpp>

using MoveArm = petra_core::action::MoveArm;

class ArmMovement : public RosAction<MoveArm>
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<float>("x"),
                                                   BT::InputPort<float>("y"),
                                                   BT::InputPort<float>("z"),
                                                   BT::InputPort<float>("gripper_position")}; }

    ArmMovement(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string actionServerName() { return "MoveArm"; }

    void onSend(MoveArm::Goal &goal) override;
    void onFeedback(const std::shared_ptr<const MoveArm::Feedback> feedback) override;
    void onResult(const rclcpp_action::ClientGoalHandle<MoveArm>::WrappedResult &result, const MoveArm::Goal &goal) override;
};