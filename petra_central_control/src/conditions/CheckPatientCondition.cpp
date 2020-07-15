#include <petra_central_control/conditions/CheckPatientCondition.h>

CheckPatientCondition::CheckPatientCondition(const std::string &name, const BT::NodeConfiguration &config) : Condition(name, config)
{
    condition_subscription_ = get_node_handle()->create_subscription<petra_core::msg::PatientCondition>("PatientCondition", 10, [&](const petra_core::msg::PatientCondition::SharedPtr msg) 
    {
        patient_condition_ = msg->condition;
    });
}

BT::NodeStatus CheckPatientCondition::onCheck()
{
    if (patient_condition_ != petra_core::msg::PatientCondition::GOOD)
    {
        log("Patient check failed");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}