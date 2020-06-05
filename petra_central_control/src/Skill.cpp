#include <petra_central_control/Skill.h>

void Skill::init()
{
    if (state_ == SkillState::uninitialized)
    {
        init_();

        set_state_(SkillState::initialized);
    }
}

void Skill::start()
{
    init();

    if (state_ == SkillState::initialized)
    {
        set_state_(SkillState::active);

        //thread_ = std::thread(&Skill::start_, this);
        //thread_.detach();

        start_();
    }
}


void Skill::spin()
{
    if (state_ == SkillState::active)
    {
        spin_();
    }
}

void Skill::stop()
{
    if (state_ == SkillState::active)
    {
        set_state_(SkillState::stopped);
        
        stop_();

        set_state_(SkillState::finished);
    }
}

void Skill::finish_()
{
    if (state_ == SkillState::active)
    {
        set_state_(SkillState::finished);
    }
}

void Skill::error_()
{
    if (state_ == SkillState::active)
    {
        set_state_(SkillState::error);

        set_state_(handle_error_());
    }
}

void Skill::next_step_(std::string msg)
{
    progress_.current_step++;
    progress_.msg = msg;

    log("Step " + std::to_string(progress_.current_step) + "/" + std::to_string(progress_.steps) + ": " + msg);
}

void Skill::set_state_(SkillState state)
{
    state_ = state;

    log(state_.to_string());
}