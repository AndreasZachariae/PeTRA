#pragma once

#include <petra_central_control/Skill.h>

class Sequence : public Skill
{
public:
    Sequence(std::shared_ptr<rclcpp::Node> node_handle);

    void spin_() override;

    std::vector<std::shared_ptr<Skill>> child_skills;
    
private:
    std::shared_ptr<rclcpp::Node> node_handle_;
};