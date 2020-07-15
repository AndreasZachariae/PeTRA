#include <petra_central_control/actions/BaseMovementRequestGoal.h>

BaseMovementRequestGoal::BaseMovementRequestGoal(const std::string &name, const BT::NodeConfiguration &config) : Action(name, config)
{
}

BT::NodeStatus BaseMovementRequestGoal::onStart()
{ 
    log("Action STARTED");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BaseMovementRequestGoal::onRunning()
{
    if (!request_sent_)
    {
        goal_point_dialog_ = UserDialog("Goal position", "Input goal coordinates x and y");
        goal_point_dialog_.add_float_key("X", -10, 10);
        goal_point_dialog_.add_float_key("Y", -10, 10);

        if (goal_point_dialog_.send_dialog([this]() {
                //data_values ist nicht voll belegt wenn Communication durch /stop resetted wurde
                if (goal_point_dialog_.get_response()->data_values.size() == 2)
                {
                    setOutput<double>("X", (double)stof(goal_point_dialog_.get_response()->data_values.at(0)));
                    setOutput<double>("Y", (double)stof(goal_point_dialog_.get_response()->data_values.at(1)));

                    request_recieved_ = true;
                }
            }))
        {
            request_sent_ = true;
        }
    }
    if (request_recieved_)
    {
        log("SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void BaseMovementRequestGoal::onHalted()
{
}