#include <petra_central_control/MoveRobot.h>

#include <petra_central_control/CentralControlUnit.h>

MoveRobot::MoveRobot(std::shared_ptr<rclcpp::Node> node_handle) : Skill("MoveRobot"), node_handle_(node_handle)
{
    progress_.steps = 5;

    stop_subscription_ = node_handle_->create_subscription<std_msgs::msg::Empty>("Stop", 10, [&](const std_msgs::msg::Empty::SharedPtr msg)
    {
        stop();
    });
    
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
        node_handle_->get_node_base_interface(),
        node_handle_->get_node_graph_interface(),
        node_handle_->get_node_logging_interface(),
        node_handle_->get_node_waitables_interface(),
        "NavigateToPose");
}

void MoveRobot::spin_()
{
    switch (progress_.current_step)
    {
        case 0:
            request_data_();
            break;
        case 2:
            send_goal_();
            break;
        case 5:
            finish_();
            break;
    }
}

void MoveRobot::stop_()
{
    log("Stop recieved", LogLevel::Warn);

    try
    {
        auto cancel_result_future = client_ptr_->async_cancel_goal(goal_handle_future_.get());
    }
    catch (const std::future_error &e)
    {
        log("Canceled before goal was sent");
    }
}

void MoveRobot::request_data_()
{
    goal_point_dialogue_ = Dialogue(node_handle_, "Goal position", "Input goal coordinates x and y");
    goal_point_dialogue_.add_float_key("X", -10, 10);
    goal_point_dialogue_.add_float_key("Y", -10, 10);

    if (goal_point_dialogue_.send_dialogue([this]() 
        {
            //data_values ist nicht voll belegt wenn Communication durch /stop resetted wurde
            if (goal_point_dialogue_.get_response()->data_values.size() == 2)
            {
                goal_point_ = new Point(
                    stof(goal_point_dialogue_.get_response()->data_values.at(0)),
                    stof(goal_point_dialogue_.get_response()->data_values.at(1)));

                next_step_("Received data...");
            }
        }))
    {
        next_step_("Requesting data...");
    }
}

void MoveRobot::send_goal_()
{
    if (client_ptr_->action_server_is_ready())
    {
        auto goal_msg = NavigateToPose::Goal();

        //get this value from user input
        goal_msg.pose.pose.position.x = goal_point_->x;
        goal_msg.pose.pose.position.y = goal_point_->y;
        goal_msg.pose.pose.position.z = 0;
        goal_msg.pose.pose.orientation.w = 1; //hard-coded orientation (x=0, y=0, z=0, w=1)
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = node_handle_->now();
        double start_time = node_handle_->now().seconds();

        log("Sending goal (x=" + ftos(goal_point_->x) + ", y=" + ftos(goal_point_->y) + ")");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback = [&](std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
        {
            if (!future.get())
            {
                log("Goal was rejected by server", LogLevel::Error);
            }

            next_step_("Goal accepted...");
        };

        send_goal_options.feedback_callback = [this](GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            log("Current position: (x=" + ftos((float)feedback->current_pose.pose.position.x) + ", y=" + ftos((float)feedback->current_pose.pose.position.y) + "), Time elapsed: " + std::to_string(feedback->navigation_time.sec) + "s");
        };

        send_goal_options.result_callback = [this, start_time](const GoalHandleNavigateToPose::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::ABORTED || result.code == rclcpp_action::ResultCode::CANCELED)
            {
                error_();
            }
            else
            {
                //result message is std_msgs/Empty

                log("Goal reached! (x=" + ftos(goal_point_->x) + ", y=" + ftos(goal_point_->y) + "), Total time: " + std::to_string((int)(node_handle_->now().seconds() - start_time)) + "s");
                
                next_step_("Received result...");
            }
        };

        goal_handle_future_ = client_ptr_->async_send_goal(goal_msg, send_goal_options);

        next_step_("Sending goal...");
    }
}