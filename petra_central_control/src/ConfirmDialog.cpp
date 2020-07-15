#include <petra_central_control/ConfirmDialog.h>

ConfirmDialog::ConfirmDialog(std::shared_ptr<rclcpp::Node> node_handle, std::string confirm_msg) 
: Skill(node_handle, "ConfirmDialog"), BT::CoroActionNode("not used", {}), node_handle_(node_handle), confirm_msg_(confirm_msg)
{
}

ConfirmDialog::ConfirmDialog(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<rclcpp::Node> node_handle, std::string confirm_msg) 
: Skill(node_handle, "ConfirmDialog"), BT::CoroActionNode(name, config), node_handle_(node_handle), confirm_msg_(confirm_msg)
{
}

//tick is executed asynchronous in an extra thread
BT::NodeStatus ConfirmDialog::tick()
{
    while (rclcpp::ok() && (get_state() != SkillState::succeeded) && (get_state() != SkillState::failed))
    {
        setStatusRunningAndYield();
    }

    if (get_state() == SkillState::succeeded)
    {
        log("[BehaviorTree] " + name() + " " + get_state().to_string());
        return BT::NodeStatus::SUCCESS;
    }

    if (get_state() == SkillState::failed)
    {
        log("[BehaviorTree] " + name() + " " + get_state().to_string());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}

void ConfirmDialog::halt()
{
    //log("HALT REQUESTED!");
    CoroActionNode::halt();
}

void ConfirmDialog::init_()
{
    progress_.steps = 2;
}

void ConfirmDialog::spin_()
{
    switch (progress_.current_step)
    {
        case 0:
            confirm_();
            break;
        case 2:
            if (confirmed_)
            {
                succeed_();
            }
            else
            {
                fail_();
            }
            break;
    }
}

void ConfirmDialog::confirm_()
{   
    confirm_dialog_ = UserDialog(node_handle_, "Please confirm", confirm_msg_);
    confirm_dialog_.add_bool_key("confirm", false);

    if (confirm_dialog_.send_dialog([this]() 
        {
            //data_value ist leer wenn Communication durch /stop resetted wurde
            if (!confirm_dialog_.get_response()->data_values.empty())
            {
                if(confirm_dialog_.get_response()->data_values.at(0) == "true")
                {
                    confirmed_ = true;
                }
                else
                {
                    confirmed_ = false;
                }

                next_step_("Received confirmation...");
            }
        }))
    {
        next_step_("Wait for confirmation...");
    }
}