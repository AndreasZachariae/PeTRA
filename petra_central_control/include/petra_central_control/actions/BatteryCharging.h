/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Provides the ROS2-Action client "ChargeBattery"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <petra_central_control/actions/base/RosAction.h>
#include <petra_central_control/actions/base/converts.h>
#include <petra_core/action/charge_battery.hpp>

using ChargeBattery = petra_core::action::ChargeBattery;

class BatteryCharging : public RosAction<ChargeBattery>
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<float>("goal_percentage")}; }

    BatteryCharging(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string actionServerName() { return "ChargeBattery"; }

    void onSend(ChargeBattery::Goal &goal) override;
    void onFeedback(const std::shared_ptr<const ChargeBattery::Feedback> feedback) override;
    void onResult(const rclcpp_action::ClientGoalHandle<ChargeBattery>::WrappedResult &result, const ChargeBattery::Goal &goal) override;
};