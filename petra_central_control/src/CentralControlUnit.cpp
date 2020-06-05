#include <petra_central_control/CentralControlUnit.h>

#include <petra_central_control/SystemMonitor.h>
#include <petra_central_control/SkillSelection.h>

CentralControlUnit::CentralControlUnit(int argc, char **argv) : Component("CCU"), state_(CCUState::uninitialized)
{
    node_handle = rclcpp::Node::make_shared("CentralControlUnit");
}

CentralControlUnit::~CentralControlUnit()
{
    log("ROS shutdown");
}

void CentralControlUnit::init()
{
    add_skill(std::make_shared<SystemMonitor>(), true);

    set_state_(CCUState::initialized);
}

void CentralControlUnit::run()
{
    if (state_ == CCUState::uninitialized)
    {
        init();
    }

    set_state_(CCUState::active);

    while ((state_ == CCUState::active) && rclcpp::ok())
    {
        if (skill_queue_.empty())
        {
            add_skill(std::make_shared<SkillSelection>(this));
        }

        if (!spin_skill_(skill_queue_.front()))
        {
            skill_queue_.pop();
        }

        for (uint index = 0; index < background_skills_.size(); ++index)
        {
            if (!spin_skill_(background_skills_[index]))
            {
                background_skills_.erase(background_skills_.begin() + (index--));
            }
        }
        
        rclcpp::spin_some(node_handle);
    }

    set_state_(CCUState::finished);
}

bool CentralControlUnit::spin_skill_(std::shared_ptr<Skill> skill_ptr)
{
    if (skill_ptr->get_state() == SkillState::finished)
    {
        return false;
    }

    if ((skill_ptr->get_state() == SkillState::uninitialized) | (skill_ptr->get_state() == SkillState::initialized))
    {
        skill_ptr->start();
    }

    skill_ptr->spin();

    return true;
}

void CentralControlUnit::add_skill(std::shared_ptr<Skill> skill_ptr, bool background)
{
    if (background)
    {
        background_skills_.push_back(skill_ptr);
        skill_ptr.get()->start();
    }
    else
    {
        skill_queue_.push(skill_ptr);
    }
}

void CentralControlUnit::set_state_(CCUState state)
{
    state_ = state;

    log(state_.to_string());
}