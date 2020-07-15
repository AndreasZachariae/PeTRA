#include <petra_central_control/CentralControlUnit.h>

#include <petra_central_control/SystemMonitor.h>
#include <petra_central_control/SkillSelection.h>

CentralControlUnit::CentralControlUnit() : Component("CCU"), state_(CCUState::uninitialized)
{
    node_handle = rclcpp::Node::make_shared("CentralControlUnit");
}

CentralControlUnit::~CentralControlUnit()
{
    log("ROS shutdown");
}

void CentralControlUnit::init()
{
    //add_skill(std::make_shared<SkillSelection>(this));

    add_skill(std::make_shared<SystemMonitor>(this), BACKGROUND);

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
        if (!mission_queue_.empty())
        {
            if (!spin_skill_(mission_queue_.front()))
            {
                mission_queue_.pop();
            }
        }

        if (!skill_queue_.empty())
        {
            if (!spin_skill_(skill_queue_.front()))
            {
                skill_queue_.pop();
            }
        }
        else
        {
            add_skill(std::make_shared<SkillSelection>(this));
        }
        
        for (uint index = 0; index < background_skills_.size(); ++index)
        {
            if (!spin_skill_(background_skills_[index]))
            {
                background_skills_.erase(background_skills_.begin() + (index--));
            }
        }
        
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(node_handle);
    }

    if (state_ != CCUState::finished)
    {
        set_state_(CCUState::finished);
    }
    
}

bool CentralControlUnit::spin_skill_(std::shared_ptr<Skill> skill_ptr)
{
    if ((skill_ptr->get_state() == SkillState::succeeded) || (skill_ptr->get_state() == SkillState::failed))
    {
        return false;
    }

    if ((skill_ptr->get_state() == SkillState::uninitialized) || (skill_ptr->get_state() == SkillState::initialized))
    {
        skill_ptr->start();
    }

    skill_ptr->spin();

    return true;
}

void CentralControlUnit::add_skill(std::shared_ptr<Skill> skill_ptr, skill_type type)
{
    switch (type)
    {
    case NORMAL:
        skill_queue_.push(skill_ptr);
        break;
    case BACKGROUND:
        background_skills_.push_back(skill_ptr);
        break;
    case MISSION:
        mission_queue_.push(skill_ptr);
        break;
    default:
        break;
    }
}

void CentralControlUnit::set_state_(CCUState state)
{
    state_ = state;

    log(state_.to_string());
}

void CentralControlUnit::pop_all_skills()
{
    while (!skill_queue_.empty())
    {
        skill_queue_.pop();
    }
}