#pragma once

#include <memory>

#include <rclcpp_action/rclcpp_action.hpp>

#include <petra_core/action/pair_device.hpp>

#include <petra_central_control/default.h>
#include <petra_central_control/Skill.h>
#include <petra_central_control/UserDialog.h>
#include <petra_central_control/DeviceType.h>

using PairDevice = petra_core::action::PairDevice;
using GoalHandlePairDevice = rclcpp_action::ClientGoalHandle<PairDevice>;

class DevicePairing : public Skill
{
public:
    DevicePairing(std::shared_ptr<rclcpp::Node> node_handle);

protected:
    void spin_() override;
    void stop_() override;

private:
    std::shared_ptr<rclcpp::Node> node_handle_;
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