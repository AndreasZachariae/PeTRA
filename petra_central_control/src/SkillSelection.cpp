#include <petra_central_control/SkillSelection.h>

#include <petra_central_control/BaseMovement.h>
#include <petra_central_control/ArmMovement.h>
#include <petra_central_control/DevicePairing.h>
#include <petra_central_control/BatteryCharging.h>
#include <petra_central_control/FreeWalking.h>
#include <petra_central_control/MissionTest.h>

SkillSelection::SkillSelection(const std::string& name, const BT::NodeConfiguration& config, CentralControlUnit* ccu) 
: Skill(ccu->node_handle, "SkillSelection"), BT::AsyncActionNode(name, config), ccu_ptr_(ccu)
{
}

SkillSelection::SkillSelection(CentralControlUnit* ccu) 
: Skill(ccu->node_handle, "SkillSelection"), BT::AsyncActionNode("not used",{}), ccu_ptr_(ccu)
{
}

BT::NodeStatus SkillSelection::tick()
{
    /*
    halt_requested_.store(false);
    
    //while tick() is blocked, BT::NodeStatus = RUNNING
    while(!halt_requested_ && rclcpp::ok() && (get_state() != SkillState::finished))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return halt_requested_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    */

   log("SkillSelection should not be used as behavior Action Node because it would be possible to recursevly call other behavior trees, which is undefined behavior");

   return BT::NodeStatus::FAILURE;
}

void SkillSelection::halt()
{
        halt_requested_.store(true);
}

void SkillSelection::init_()
{
    progress_.steps = 2;

    default_options_.push_back("BaseMovement");
    default_options_.push_back("ArmMovement");
    default_options_.push_back("DevicePairing");
    default_options_.push_back("BatteryCharging");
    default_options_.push_back("MissionTest");
    default_options_.push_back("FreeWalking");
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
            succeed_();
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
    else if (skill_str == "MissionTest")
    {
        ccu_ptr_->add_skill(std::make_shared<MissionTest>(ccu_ptr_->node_handle));
    }
    else if (skill_str == "FreeWalking")
    {
        ccu_ptr_->add_skill(std::make_shared<FreeWalking>(ccu_ptr_), MISSION);
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