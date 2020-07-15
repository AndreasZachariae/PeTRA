#include <petra_central_control/actions/BatteryCharging.h>

BatteryCharging::BatteryCharging(const std::string &name, const BT::NodeConfiguration &config) : Action(name, config)
{
    progress_.steps = 5;
}

BT::NodeStatus BatteryCharging::onStart()
{
    charge_battery_client_ = rclcpp_action::create_client<ChargeBattery>(
        get_node_handle()->get_node_base_interface(),
        get_node_handle()->get_node_graph_interface(),
        get_node_handle()->get_node_logging_interface(),
        get_node_handle()->get_node_waitables_interface(),
        "ChargeBattery");
        
    log("Action STARTED");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BatteryCharging::onRunning()
{
    switch (progress_.current_step)
    {
    case Progress::FAIL_STEP:
        return BT::NodeStatus::FAILURE;
    case 0:
        request_percentage_();
        break;
    case 2:
        send_goal_();
        break;
    case 5:
        return BT::NodeStatus::SUCCESS;
        break;
    }

    return BT::NodeStatus::RUNNING;
}

void BatteryCharging::onHalted()
{
    try
    {
        auto cancel_result_future = charge_battery_client_->async_cancel_goal(goal_handle_future_.get());
    }
    catch (const std::future_error &e)
    {
        log("Canceled before goal was sent");
    }
}

void BatteryCharging::set_percentage(float goal_percentage)
{
    goal_percentage_ = goal_percentage;

    progress_.set_step(2);
}

void BatteryCharging::request_percentage_()
{
    goal_percentage_dialog_ = UserDialog("Goal battery level", "input battery level in percent %");
    goal_percentage_dialog_.add_int_key("percentage", 0, 100);

    if (goal_percentage_dialog_.send_dialog([this]() {
            //data_values ist nicht voll belegt wenn Communication durch /stop resetted wurde
            if (goal_percentage_dialog_.get_response()->data_values.size() == 1)
            {
                goal_percentage_ = std::stof(goal_percentage_dialog_.get_response()->data_values.at(0)) / 100;

                progress_.next_step("Received battery level...");
            }
        }))
    {
        progress_.next_step("Requesting battery level...");
    }
}

void BatteryCharging::send_goal_()
{
    if (charge_battery_client_->action_server_is_ready())
    {
        auto goal_msg = ChargeBattery::Goal();

        //get this value from user input
        goal_msg.goal_percentage = goal_percentage_;

        double start_time = get_node_handle()->now().seconds();

        log("Goal: " + ftos(goal_percentage_ * 100) + "%");

        auto send_goal_options = rclcpp_action::Client<ChargeBattery>::SendGoalOptions();

        send_goal_options.goal_response_callback = [&](std::shared_future<GoalHandleChargeBattery::SharedPtr> future) {
            if (!future.get())
            {
                log("Goal was rejected by server", LogLevel::Error);
                progress_.set_step(5);
            }
            else
            {
                progress_.next_step("Goal accepted...");
            }
        };

        send_goal_options.feedback_callback = [this](GoalHandleChargeBattery::SharedPtr, const std::shared_ptr<const ChargeBattery::Feedback> feedback) {
            log("Battery level: " + ftos(feedback->current_state.percentage * 100) + "%");
        };

        send_goal_options.result_callback = [this, start_time](const GoalHandleChargeBattery::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::ABORTED || result.code == rclcpp_action::ResultCode::CANCELED || result.code == rclcpp_action::ResultCode::UNKNOWN)
            {
                progress_.set_fail();
            }
            else
            {
                if (result.result->success)
                {
                    log("Finished! Total time: " + std::to_string((int)(get_node_handle()->now().seconds() - start_time)) + "s");
                }
                else
                {
                    log("Failed to charge!");
                }

                progress_.next_step("Received result...");
            }
        };

        goal_handle_future_ = charge_battery_client_->async_send_goal(goal_msg, send_goal_options);

        progress_.next_step("Sending goal...");
    }
}