#pragma once

#include <petra_central_control/default.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <petra_central_control/actions/base/Action.h>
#include <petra_central_control/actions/types/UserDialog.h>
#include <petra_central_control/actions/types/DeviceType.h>
#include <petra_core/action/pair_device.hpp>

using PairDevice = petra_core::action::PairDevice;
using GoalHandlePairDevice = rclcpp_action::ClientGoalHandle<PairDevice>;

class DevicePairing : public Action
{
public:
    static BT::PortsList providedPorts() { return BT::PortsList(); }

    DevicePairing(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    void set_mode(int pairing_mode);
    void set_mode_and_device(int pairing_mode, DeviceType device);

private:
    rclcpp_action::Client<PairDevice>::SharedPtr pair_device_client_;
    std::shared_future<std::shared_ptr<GoalHandlePairDevice>> goal_handle_future_;

    UserDialog device_type_dialog_;
    UserDialog pairing_mode_dialog_;
    DeviceType device_;

    int pairing_mode_;

    void request_pairing_mode_();
    void request_device_type_();
    void start_pairing_();
    void start_unpairing_();

    void send_action_goal_(PairDevice::Goal goal);
};