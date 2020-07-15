#include <petra_central_control/actions/BaseMovement.h>

BaseMovement::BaseMovement(const std::string &name, const BT::NodeConfiguration &config) : Action(name, config)
{
    progress_.steps = 5;
}

BT::NodeStatus BaseMovement::onStart()
{
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
        get_node_handle()->get_node_base_interface(),
        get_node_handle()->get_node_graph_interface(),
        get_node_handle()->get_node_logging_interface(),
        get_node_handle()->get_node_waitables_interface(),
        "NavigateToPose");
        
    log("Action STARTED");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BaseMovement::onRunning()
{
    switch (progress_.current_step)
    {
    case Progress::FAIL_STEP:
        return BT::NodeStatus::FAILURE;
    case 0:
        request_location_();
        break;
    case 2:
        send_goal_();
        break;
    case 5:
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void BaseMovement::onHalted()
{
    //log("HALT REQUESTED!");
    try
    {
        auto cancel_result_future = navigate_to_pose_client_->async_cancel_goal(goal_handle_future_.get());
    }
    catch (const std::future_error &e)
    {
        log("Canceled before goal was sent");
    }
}

void BaseMovement::set_goal(Point goal_point)
{
    goal_point_ = goal_point;

    progress_.set_step(2);
}

void BaseMovement::request_location_()
{
    goal_point_dialog_ = UserDialog("Goal position", "Input goal coordinates x and y");
    goal_point_dialog_.add_float_key("X", -10, 10);
    goal_point_dialog_.add_float_key("Y", -10, 10);

    if (goal_point_dialog_.send_dialog([this]() {
            //data_values ist nicht voll belegt wenn Communication durch /stop resetted wurde
            if (goal_point_dialog_.get_response()->data_values.size() == 2)
            {
                goal_point_ = Point(
                    stof(goal_point_dialog_.get_response()->data_values.at(0)),
                    stof(goal_point_dialog_.get_response()->data_values.at(1)));

                progress_.next_step("Received location...");
            }
        }))
    {
        progress_.next_step("Requesting location...");
    }
}

void BaseMovement::send_goal_()
{
    if (navigate_to_pose_client_->action_server_is_ready())
    {
        auto goal_msg = NavigateToPose::Goal();

        //get this value from user input
        goal_msg.pose.pose.position.x = goal_point_.x;
        goal_msg.pose.pose.position.y = goal_point_.y;
        goal_msg.pose.pose.position.z = 0;
        goal_msg.pose.pose.orientation.w = 1; //hard-coded orientation (x=0, y=0, z=0, w=1)
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = get_node_handle()->now();
        double start_time = get_node_handle()->now().seconds();

        log("Goal: (x=" + ftos(goal_point_.x) + ", y=" + ftos(goal_point_.y) + ")");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback = [&](std::shared_future<GoalHandleNavigateToPose::SharedPtr> future) {
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

        send_goal_options.feedback_callback = [this](GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
            log("Current position: (x=" + ftos((float)feedback->current_pose.pose.position.x) + ", y=" + ftos((float)feedback->current_pose.pose.position.y) + "), Time elapsed: " + std::to_string(feedback->navigation_time.sec) + "s");
        };

        send_goal_options.result_callback = [this, start_time](const GoalHandleNavigateToPose::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::ABORTED || result.code == rclcpp_action::ResultCode::CANCELED || result.code == rclcpp_action::ResultCode::UNKNOWN)
            {
                progress_.set_fail();
            }
            else
            {
                //result message is std_msgs/Empty

                log("Goal reached! (x=" + ftos(goal_point_.x) + ", y=" + ftos(goal_point_.y) + "), Total time: " + std::to_string((int)(get_node_handle()->now().seconds() - start_time)) + "s");

                progress_.next_step("Received result...");
            }
        };

        goal_handle_future_ = navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);

        progress_.next_step("Sending goal...");
    }
}