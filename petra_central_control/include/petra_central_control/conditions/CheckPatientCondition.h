#pragma once

#include <petra_central_control/default.h>

#include <petra_central_control/conditions/base/Condition.h>
#include <petra_core/msg/patient_condition.hpp>

class CheckPatientCondition : public Condition
{
public:
    static BT::PortsList providedPorts() { return BT::PortsList(); }

    CheckPatientCondition(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onCheck() override;

private:
    rclcpp::Subscription<petra_core::msg::PatientCondition>::SharedPtr condition_subscription_;

    unsigned char patient_condition_ = petra_core::msg::PatientCondition::GOOD;
};
