#include <petra_central_control/SkillSelection.h>
#include <petra_central_control/BaseMovement.h>
#include <petra_central_control/ArmMovement.h>
#include <petra_central_control/DevicePairing.h>
#include <petra_central_control/BatteryCharging.h>

SkillSelection::SkillSelection(CentralControlUnit* ccu) : Skill(ccu->node_handle, "SkillSelection"), ccu_ptr_(ccu)
{
    progress_.steps = 2;
}

void SkillSelection::init_()
{
    default_options_.push_back("BaseMovement");
    default_options_.push_back("ArmMovement");
    default_options_.push_back("DevicePairing");
    default_options_.push_back("BatteryCharging");
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

void SkillSelection::select_skill_()
{   
    std::string msg = "";
    for (size_t i = 0; i < default_options_.size(); i++)
    {
        msg = msg + "[" + std::to_string(i+1) + "] " + default_options_.at(i) + "\n";
    }
    msg.pop_back();
    
    selection_dialog_ = UserDialog(ccu_ptr_->node_handle, "Select a skill", msg);
    selection_dialog_.add_int_key("index", 1, default_options_.size(), 0);

    if (selection_dialog_.send_dialog([this]() 
        {
            //data_value ist leer wenn Communication durch /stop resetted wurde
            if (!selection_dialog_.get_response()->data_values.empty())
            {
                int index = std::stoi(selection_dialog_.get_response()->data_values.at(0));

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
    if (skill_str == "BaseMovement")
    {
        ccu_ptr_->add_skill(std::make_shared<BaseMovement>(ccu_ptr_->node_handle));
    }
    else if (skill_str == "ArmMovement")
    {
        ccu_ptr_->add_skill(std::make_shared<ArmMovement>(ccu_ptr_->node_handle));
    }
    else if (skill_str == "DevicePairing")
    {
        ccu_ptr_->add_skill(std::make_shared<DevicePairing>(ccu_ptr_->node_handle));
    }
    else if (skill_str == "BatteryCharging")
    {
        ccu_ptr_->add_skill(std::make_shared<BatteryCharging>(ccu_ptr_->node_handle));
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