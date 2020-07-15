#include <petra_central_control/actions/SkillSelection.h>

#include <petra_central_control/actions/BaseMovement.h>
#include <petra_central_control/actions/ArmMovement.h>
#include <petra_central_control/actions/DevicePairing.h>
#include <petra_central_control/actions/BatteryCharging.h>

SkillSelection::SkillSelection(const std::string &name, const BT::NodeConfiguration &config) : Action(name, config)
{
    progress_.steps = 2;

    default_options_.push_back("DevicePairing");
    default_options_.push_back("BaseMovement");
    default_options_.push_back("ArmMovement");
    default_options_.push_back("BatteryCharging");
    default_options_.push_back("FreeWalking");
    default_options_.push_back("GuidedWalking");
    default_options_.push_back("TransportWithDevice");
}

BT::NodeStatus SkillSelection::onRunning()
{
    switch (progress_.current_step)
    {
    case Progress::FAIL_STEP:
        return BT::NodeStatus::FAILURE;
    case 0:
        select_skill_();
        break;
    case 2:
        return BT::NodeStatus::SUCCESS;
        break;
    }

    return BT::NodeStatus::RUNNING;
}

void SkillSelection::onHalted()
{
}

void SkillSelection::select_skill_()
{
    std::string msg = "";
    for (size_t i = 0; i < default_options_.size(); i++)
    {
        msg = msg + "[" + std::to_string(i + 1) + "] " + default_options_.at(i) + "\n";
    }
    msg.pop_back();

    selection_dialog_ = UserDialog("Select a skill", msg);
    selection_dialog_.add_int_key("index", 1, default_options_.size(), 0);

    if (selection_dialog_.send_dialog([this]() {
            //data_value ist leer wenn Communication durch /stop resetted wurde
            if (!selection_dialog_.get_response()->data_values.empty())
            {
                int index = std::stoi(selection_dialog_.get_response()->data_values.at(0));

                std::string skill_str = default_options_.at(index - 1);

                log("Selected Skill: [" + skill_str + "]");

                setOutput("selected_action", skill_str);

                //queue_chosen_skill_(skill_str);

                progress_.next_step("Received selection...");
            }
        }))
    {
        progress_.next_step("Sending selection...");
    }
}

void SkillSelection::queue_chosen_skill_(std::string skill_str)
{
    if (skill_str == "BaseMovement")
    {
    }
    else if (skill_str == "ArmMovement")
    {
    }
    else if (skill_str == "DevicePairing")
    {
    }
    else if (skill_str == "BatteryCharging")
    {
    }
    else if (skill_str == "MissionTest")
    {
    }
    else if (skill_str == "FreeWalking")
    {
    }
    else if (skill_str == "Shutdown CCU")
    {
    }
    else
    {
        log("Invalid skill name", LogLevel::Warn);
    }
}