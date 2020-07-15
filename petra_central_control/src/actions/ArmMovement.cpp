#include <petra_central_control/actions/ArmMovement.h>

ArmMovement::ArmMovement(const std::string &name, const BT::NodeConfiguration &config) : Action(name, config)
{
    progress_.steps = 5;
}

BT::NodeStatus ArmMovement::onStart()
{

    move_arm_client_ = rclcpp_action::create_client<MoveArm>(
        get_node_handle()->get_node_base_interface(),
        get_node_handle()->get_node_graph_interface(),
        get_node_handle()->get_node_logging_interface(),
        get_node_handle()->get_node_waitables_interface(),
        "MoveArm");

    log("Action STARTED");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmMovement::onRunning()
{
    switch (progress_.current_step)
    {
    case Progress::FAIL_STEP:
        return BT::NodeStatus::FAILURE;
    case 0:
        request_coordinates_();
        break;
    case 2:
        send_goal_();
        break;
    case 5:
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void ArmMovement::onHalted()
{
    try
    {
        auto cancel_result_future = move_arm_client_->async_cancel_goal(goal_handle_future_.get());
    }
    catch (const std::future_error &e)
    {
        log("Canceled before goal was sent");
    }
}

void ArmMovement::set_coordinates(Point goal_point, float gripper_position)
{
    goal_point_ = goal_point;
    gripper_position_ = gripper_position;

    progress_.current_step = 2;
}

void ArmMovement::request_coordinates_()
{
    goal_point_dialog_ = UserDialog("Goal coordinates", "coordinates x,y,z in [mm] and \ngripper position from 0 (open) to 1 (closed)");
    goal_point_dialog_.add_float_key("X", -1000, 1000);
    goal_point_dialog_.add_float_key("Y", -1000, 1000);
    goal_point_dialog_.add_float_key("Z", -1000, 1000);
    goal_point_dialog_.add_float_key("gripper_position", 0, 1);

    if (goal_point_dialog_.send_dialog([this]() {
            //data_values ist nicht voll belegt wenn Communication durch /stop resetted wurde
            if (goal_point_dialog_.get_response()->data_values.size() == 4)
            {
                goal_point_ = Point(
                    stof(goal_point_dialog_.get_response()->data_values.at(0)),
                    stof(goal_point_dialog_.get_response()->data_values.at(1)),
                    stof(goal_point_dialog_.get_response()->data_values.at(2)));

                gripper_position_ = stof(goal_point_dialog_.get_response()->data_values.at(3));

                progress_.next_step("Received coordinates...");
            }
        }))
    {
        progress_.next_step("Requesting coordinates...");
    }
}

void ArmMovement::send_goal_()
{
    if (move_arm_client_->action_server_is_ready())
    {
        auto goal_msg = MoveArm::Goal();

        //get this value from user input
        goal_msg.pose.position.x = goal_point_.x;
        goal_msg.pose.position.y = goal_point_.y;
        goal_msg.pose.position.z = goal_point_.z;
        goal_msg.pose.position.z = 0;
        goal_msg.pose.orientation.w = 1; //hard-coded orientation (x=0, y=0, z=0, w=1)
        goal_msg.gripper_position = gripper_position_;
        double start_time = get_node_handle()->now().seconds();

        log("Goal: (x=" + ftos(goal_point_.x) + ", y=" + ftos(goal_point_.y) + ", z=" + ftos(goal_point_.z) + ")");

        auto send_goal_options = rclcpp_action::Client<MoveArm>::SendGoalOptions();

        send_goal_options.goal_response_callback = [&](std::shared_future<GoalHandleMoveArm::SharedPtr> future) {
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

        send_goal_options.feedback_callback = [this](GoalHandleMoveArm::SharedPtr, const std::shared_ptr<const MoveArm::Feedback> feedback) {
            log("Position: (x=" + ftos((float)feedback->current_pose.position.x) + ", y=" + ftos((float)feedback->current_pose.position.y) + ", z=" + ftos((float)feedback->current_pose.position.z) + "), Progress: " + ftos((float)(feedback->progress * 100)) + "%, Time: " + std::to_string(feedback->movement_time.sec) + "s");
        };

        send_goal_options.result_callback = [this, start_time](const GoalHandleMoveArm::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::ABORTED || result.code == rclcpp_action::ResultCode::CANCELED || result.code == rclcpp_action::ResultCode::UNKNOWN)
            {
                progress_.set_fail();
            }
            else
            {
                if (result.result->success)
                {
                    log("Goal reached! (x=" + ftos(goal_point_.x) + ", y=" + ftos(goal_point_.y) + ", z=" + ftos(goal_point_.z) + "), Total time: " + std::to_string((int)(get_node_handle()->now().seconds() - start_time)) + "s");
                }
                else
                {
                    log("Goal not reached!");
                }

                progress_.next_step("Received result...");
            }
        };

        goal_handle_future_ = move_arm_client_->async_send_goal(goal_msg, send_goal_options);

        progress_.next_step("Sending goal...");
    }
}