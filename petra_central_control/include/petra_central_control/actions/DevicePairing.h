/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Provides the ROS2-Action client "PairDevice"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <petra_central_control/actions/base/RosAction.h>
#include <petra_central_control/actions/base/converts.h>
#include <petra_core/action/pair_device.hpp>

using PairDevice = petra_core::action::PairDevice;

class DevicePairing : public RosAction<PairDevice>
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<int>("pairing_mode"),
                                                   BT::InputPort<int>("device_type")}; }

    DevicePairing(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string actionServerName() { return "PairDevice"; }

    void onSend(PairDevice::Goal &goal) override;
    void onFeedback(const std::shared_ptr<const PairDevice::Feedback> feedback) override;
    void onResult(const rclcpp_action::ClientGoalHandle<PairDevice>::WrappedResult &result, const PairDevice::Goal &goal) override;
};