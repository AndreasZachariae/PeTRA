#include <petra_central_control/Skill.h>

Skill::Skill(std::shared_ptr<rclcpp::Node> node_handle, std::string name, bool background)
: Component(name), node_handle_(node_handle)
{
    if (!background)
    {
        stop_subscription_ = node_handle_->create_subscription<std_msgs::msg::Empty>("Stop", 10, [&](const std_msgs::msg::Empty::SharedPtr msg) 
        {
            //log("Stop recieved", LogLevel::Error);
            stop();
        });
    }
}

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

void Skill::set_step_(int step)
{
    progress_.current_step = step;
}

void Skill::set_state_(SkillState state)
{
    state_ = state;

    log(state_.to_string());
}