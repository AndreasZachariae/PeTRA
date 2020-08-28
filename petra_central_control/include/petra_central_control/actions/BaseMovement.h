/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Provides the ROS2-Action client "NavigateToPose"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <petra_central_control/actions/base/RosAction.h>
#include <petra_central_control/actions/base/converts.h>
#include <petra_core/action/navigate_to_pose.hpp>

using NavigateToPose = petra_core::action::NavigateToPose;

class BaseMovement : public RosAction<NavigateToPose>
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<float>("x"),
                                                   BT::InputPort<float>("y")}; }

    BaseMovement(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string actionServerName() { return "NavigateToPose"; }

    void onSend(NavigateToPose::Goal &goal) override;
    void onFeedback(const std::shared_ptr<const NavigateToPose::Feedback> feedback) override;
    void onResult(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result, const NavigateToPose::Goal &goal) override;
};