#pragma once

#include <petra_central_control/CentralControlUnit.h>

#include <petra_core/Point.h>
#include <petra_core/msg/patient_condition.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>

class FreeWalking : public Skill
{
public:
    FreeWalking(CentralControlUnit* ccu);

    BT::NodeStatus check_condition();

protected:
    void init_() override;
    void spin_() override;
    void stop_() override;

private:
    rclcpp::Subscription<petra_core::msg::PatientCondition>::SharedPtr condition_subscription_;

    CentralControlUnit* ccu_ptr_ = nullptr;
    BT::Tree behavior_tree_;

    int patient_condition_ = petra_core::msg::PatientCondition::GOOD;

    Point room1_ = Point(5,5);
    Point room2_ = Point(6,-9);
    Point waiting_area_ = Point(-9,-9);
};