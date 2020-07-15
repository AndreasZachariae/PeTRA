#include <petra_central_control/actions/ConfirmDialog.h>

ConfirmDialog::ConfirmDialog(const std::string &name, const BT::NodeConfiguration &config) : Action(name, config)
{
    progress_.steps = 2;

    confirm_msg = "[y]yes [n]no";
}

BT::NodeStatus ConfirmDialog::onRunning()
{
    switch (progress_.current_step)
    {
    case Progress::FAIL_STEP:
        return BT::NodeStatus::FAILURE;
    case 0:
        confirm_();
        break;
    case 2:
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void ConfirmDialog::onHalted()
{
}

void ConfirmDialog::confirm_()
{
    confirm_dialog_ = UserDialog(getInput<std::string>("message").value(), confirm_msg, petra_core::srv::UserDialog::Request::HIGH);
    confirm_dialog_.add_bool_key("confirm", false);

    if (confirm_dialog_.send_dialog([this]() {
            //data_value ist leer wenn Communication durch /stop resetted wurde
            if (!confirm_dialog_.get_response()->data_values.empty())
            {
                if (confirm_dialog_.get_response()->data_values.at(0) == "true")
                {
                    progress_.next_step("Received confirmation...");
                }
                else
                {
                    progress_.set_fail();
                }
            }
        }))
    {
        progress_.next_step("Wait for confirmation...");
    }
}