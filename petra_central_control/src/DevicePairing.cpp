#include <petra_central_control/DevicePairing.h>

DevicePairing::DevicePairing(std::shared_ptr<rclcpp::Node> node_handle) : Skill(node_handle, "DevicePairing"), node_handle_(node_handle)
{
    progress_.steps = 7;
    
    pair_device_client_ = rclcpp_action::create_client<PairDevice>(
        node_handle_->get_node_base_interface(),
        node_handle_->get_node_graph_interface(),
        node_handle_->get_node_logging_interface(),
        node_handle_->get_node_waitables_interface(),
        "PairDevice");
}

void DevicePairing::spin_()
{
    switch (progress_.current_step)
    {
        case 0:
            request_pairing_mode_();
            break;
        case 2:
            if (pairing_mode_ == PairDevice::Goal::UNPAIR)
            {
                set_step_(4);
            }
            else
            {
                request_device_type_();
            }
            break;
        case 4:
            if (pairing_mode_ == PairDevice::Goal::UNPAIR)
            {
                start_unpairing_();
            }
            else
            {
                start_pairing_();
            }
            break;
        case 7:
            finish_();
            break;
    }
}

void DevicePairing::stop_()
{
    try
    {
        auto cancel_result_future = pair_device_client_->async_cancel_goal(goal_handle_future_.get());
    }
    catch (const std::future_error &e)
    {
        log("Canceled before goal was sent");
    }
}

void DevicePairing::request_pairing_mode_()
{
    std::string msg = "[y]pair [n]unpair";

    pairing_mode_dialog_ = UserDialog(node_handle_, "Pair or unpair a device?", msg);
    pairing_mode_dialog_.add_bool_key("pairing_mode", true);

    if (pairing_mode_dialog_.send_dialog([this]() 
        {
            //data_values ist nicht voll belegt wenn Communication durch /stop resetted wurde
            if (pairing_mode_dialog_.get_response()->data_values.size() == 1)
            {
                if(pairing_mode_dialog_.get_response()->data_values.at(0) == "true")
                {
                    pairing_mode_ = PairDevice::Goal::PAIR;
                }
                else
                {
                    pairing_mode_ = PairDevice::Goal::UNPAIR;
                }
                
                next_step_("Received pairing mode...");
            }
        }))
    {
        next_step_("Requesting pairing mode...");
    }
}

void DevicePairing::request_device_type_()
{
    std::string msg = "[1]" + DeviceType(DeviceType::wheelchair).to_string()
                    + "\n[2]" + DeviceType(DeviceType::rollator).to_string()
                    + "\n[3]" + DeviceType(DeviceType::hospitalbed).to_string()
                    + "\n[4]" + DeviceType(DeviceType::chargingstation).to_string();

    device_type_dialog_ = UserDialog(node_handle_, "Select the device to pair", msg);
    device_type_dialog_.add_int_key("Index", 1, 4, 0);

    if (device_type_dialog_.send_dialog([this]() 
        {
            //data_values ist nicht voll belegt wenn Communication durch /stop resetted wurde
            if (device_type_dialog_.get_response()->data_values.size() == 1)
            {
                int index = std::stoi(device_type_dialog_.get_response()->data_values.at(0));

                switch (index)
                {
                case 1:
                    device_ = DeviceType::wheelchair;
                    break;
                case 2:
                    device_ = DeviceType::rollator;
                    break;
                case 3:
                    device_ = DeviceType::hospitalbed;
                    break;
                case 4:
                    device_ = DeviceType::chargingstation;
                    break;
                
                default:
                    break;
                }

                next_step_("Received device...");
            }
        }))
    {
        next_step_("Requesting device...");
    }
}

void DevicePairing::start_pairing_()
{
    auto goal_msg = PairDevice::Goal();

    goal_msg.device_type = device_;
    goal_msg.pairing_mode = PairDevice::Goal::PAIR;

    log("Start pairing to " + device_.to_string());
        
    send_action_goal_(goal_msg);
}

void DevicePairing::start_unpairing_()
{
    auto goal_msg = PairDevice::Goal();

    goal_msg.device_type = device_;
    goal_msg.pairing_mode = PairDevice::Goal::UNPAIR;

    log("Start unpairing of " + device_.to_string());
        
    send_action_goal_(goal_msg);
}

void DevicePairing::send_action_goal_(PairDevice::Goal goal)
{
    if (pair_device_client_->action_server_is_ready())
    {
        double start_time = node_handle_->now().seconds();

        auto send_goal_options = rclcpp_action::Client<PairDevice>::SendGoalOptions();

        send_goal_options.goal_response_callback = [&](std::shared_future<GoalHandlePairDevice::SharedPtr> future)
        {
            if (!future.get())
            {
                log("Action was rejected by server", LogLevel::Error);
                set_step_(7);
            }
            else
            {   
                next_step_("Action accepted...");
            }
        };

        send_goal_options.feedback_callback = [this](GoalHandlePairDevice::SharedPtr, const std::shared_ptr<const PairDevice::Feedback> feedback)
        {
            log("Progress: " + ftos(feedback->progress * 100) +" %");
        };

        send_goal_options.result_callback = [this, start_time](const GoalHandlePairDevice::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::ABORTED || result.code == rclcpp_action::ResultCode::CANCELED || result.code == rclcpp_action::ResultCode::UNKNOWN)
            {
                error_();
            }
            else
            {
                if (result.result->success == true)
                {
                    log("Device pairing succeeded!, Total time: " + std::to_string((int)(node_handle_->now().seconds() - start_time)) + "s");
                }
                else
                {
                    log("Device pairing failed!");
                }

                next_step_("Received result...");
            }
        };

        goal_handle_future_ = pair_device_client_->async_send_goal(goal, send_goal_options);

        next_step_("Sending action goal...");
    }
}