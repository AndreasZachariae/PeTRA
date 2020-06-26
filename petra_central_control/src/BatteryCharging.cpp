#include <petra_central_control/BatteryCharging.h>

BatteryCharging::BatteryCharging(std::shared_ptr<rclcpp::Node> node_handle) : Skill(node_handle, "BatteryCharging"), node_handle_(node_handle)
{
    progress_.steps = 5;
    
    charge_battery_client_ = rclcpp_action::create_client<ChargeBattery>(
        node_handle_->get_node_base_interface(),
        node_handle_->get_node_graph_interface(),
        node_handle_->get_node_logging_interface(),
        node_handle_->get_node_waitables_interface(),
        "ChargeBattery");
}

void BatteryCharging::spin_()
{
    switch (progress_.current_step)
    {
        case 0:
            request_percentage_();
            break;
        case 2:
            send_goal_();
            break;
        case 5:
            finish_();
            break;
    }
}

void BatteryCharging::stop_()
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

void BatteryCharging::request_percentage_()
{
    goal_percentage_dialog_ = UserDialog(node_handle_, "Goal battery level", "input battery level in percent %");
    goal_percentage_dialog_.add_int_key("percentage", 0, 100);

    if (goal_percentage_dialog_.send_dialog([this]() 
        {
            //data_values ist nicht voll belegt wenn Communication durch /stop resetted wurde
            if (goal_percentage_dialog_.get_response()->data_values.size() == 1)
            {
                goal_percentage_ = std::stof(goal_percentage_dialog_.get_response()->data_values.at(0)) / 100;

                next_step_("Received battery level...");
            }
        }))
    {
        next_step_("Requesting battery level...");
    }
}

void BatteryCharging::send_goal_()
{
    if (charge_battery_client_->action_server_is_ready())
    {
        auto goal_msg = ChargeBattery::Goal();

        //get this value from user input
        goal_msg.goal_percentage = goal_percentage_;

        double start_time = node_handle_->now().seconds();

        log("Goal: " + ftos(goal_percentage_ * 100) + "%");

        auto send_goal_options = rclcpp_action::Client<ChargeBattery>::SendGoalOptions();

        send_goal_options.goal_response_callback = [&](std::shared_future<GoalHandleChargeBattery::SharedPtr> future)
        {
            if (!future.get())
            {
                log("Goal was rejected by server", LogLevel::Error);
                set_step_(5);
            }
            else
            {   
                next_step_("Goal accepted...");
            }
        };

        send_goal_options.feedback_callback = [this](GoalHandleChargeBattery::SharedPtr, const std::shared_ptr<const ChargeBattery::Feedback> feedback)
        {
            log("Battery level: " + ftos(feedback->current_state.percentage * 100) + "%");
        };

        send_goal_options.result_callback = [this, start_time](const GoalHandleChargeBattery::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::ABORTED || result.code == rclcpp_action::ResultCode::CANCELED || result.code == rclcpp_action::ResultCode::UNKNOWN)
            {
                error_();
            }
            else
            {
                if (result.result->success == true)
                {
                    log("Finished! Total time: " + std::to_string((int)(node_handle_->now().seconds() - start_time)) + "s");
                }
                else
                {
                    log("Failed to charge!");
                }

                next_step_("Received result...");
            }
        };

        goal_handle_future_ = charge_battery_client_->async_send_goal(goal_msg, send_goal_options);

        next_step_("Sending goal...");
    }
}