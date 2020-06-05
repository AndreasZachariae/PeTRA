#include <petra_central_control/SkillSelection.h>
#include <petra_central_control/MoveRobot.h>

SkillSelection::SkillSelection(CentralControlUnit* ccu) : Skill("SkillSelection"), ccu_ptr_(ccu)
{
    progress_.steps = 2;

    stop_subscription_ = ccu_ptr_->node_handle->create_subscription<std_msgs::msg::Empty>("Stop", 10, [&](const std_msgs::msg::Empty::SharedPtr msg)
    {
        stop();
    });
}

void SkillSelection::init_()
{
    default_options_.push_back("MoveRobot");
    default_options_.push_back("skill2");
    default_options_.push_back("skill3");
    default_options_.push_back("Shutdown CCU");
}

void SkillSelection::spin_()
{
    switch (progress_.current_step)
    {
        case 0:
            select_skill_();
            break;
        case 2:
            finish_();
            break;
    }
}

void SkillSelection::stop_()
{
    log("Stop recieved", LogLevel::Warn);
}

void SkillSelection::select_skill_()
{   
    std::string msg = "Write corresponding index in Keyboard node\n";
    for (int i = 0; i < default_options_.size(); i++)
    {
        msg = msg + "[" + std::to_string(i+1) + "]" + default_options_.at(i) + " ";
    }
    
    selection_ = Dialogue(ccu_ptr_->node_handle, "Choose a skill", msg);
    selection_.add_int_key("index", 1, default_options_.size(), 0);

    if (selection_.send_dialogue([this]() 
        {
            //data_value ist leer wenn Communication durch /stop resetted wurde
            if (!selection_.get_response()->data_values.empty())
            {
                int index = std::stoi(selection_.get_response()->data_values.at(0));

                std::string skill_str = default_options_.at(index - 1);
                
                log("Selected Skill: [" + skill_str + "]");

                queue_chosen_skill_(skill_str);

                next_step_("Received selection...");
            }
        }))
    {
        next_step_("Sending selection...");
    }
}

void SkillSelection::queue_chosen_skill_(std::string skill_str)
{
    if (skill_str == "MoveRobot")
    {
        ccu_ptr_->add_skill(std::make_shared<MoveRobot>(ccu_ptr_->node_handle));
    }
    else if (skill_str == "Shutdown CCU")
    {
        ccu_ptr_->set_state_(CCUState::finished);
    }
    else
    {
        log("Invalid skill name", LogLevel::Warn);
    }
}