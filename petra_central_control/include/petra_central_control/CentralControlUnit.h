#pragma once

#include <queue>

#include <rclcpp/rclcpp.hpp>

#include <petra_central_control/CCUState.h>
#include <petra_central_control/Skill.h>

enum skill_type { NORMAL, BACKGROUND, MISSION };

class CentralControlUnit : Component
{
public:
    CentralControlUnit();
    ~CentralControlUnit();

    std::shared_ptr<rclcpp::Node> node_handle;

    void init();
    void run();
    
    void add_skill(std::shared_ptr<Skill> skill_ptr, skill_type type = NORMAL);
    void pop_all_skills();

private:
    CCUState state_;

    std::queue<std::shared_ptr<Skill>> skill_queue_;
    std::queue<std::shared_ptr<Skill>> mission_queue_;
    std::vector<std::shared_ptr<Skill>> background_skills_;

    bool spin_skill_(std::shared_ptr<Skill> skill_ptr);
    
    void set_state_(CCUState state);
    friend class SkillSelection;
};