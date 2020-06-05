#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>

#include <petra_central_control/default.h>
#include <petra_central_control/Skill.h>
#include <petra_central_control/CentralControlUnit.h>
#include <petra_central_control/Dialogue.h>

class SkillSelection : public Skill
{
private:
    std::vector<std::string> default_options_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
    CentralControlUnit* ccu_ptr_;
    Dialogue selection_;

    void select_skill_();
    void queue_chosen_skill_(std::string);
public:
    SkillSelection(CentralControlUnit* ccu);

protected:
    void init_() override;
    void spin_() override;
    void stop_() override;
};
