#include <petra_drivers/ManipulatorDummy.h>

ManipulatorDummy::ManipulatorDummy() : Node("ManipulatorDummy")
{
    move_arm_server_ = rclcpp_action::create_server<MoveArm>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        "MoveArm",
        std::bind(&ManipulatorDummy::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ManipulatorDummy::handle_cancel_, this, std::placeholders::_1),
        std::bind(&ManipulatorDummy::handle_accepted_, this, std::placeholders::_1));

    diagnostic_status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("DiagnosticStatus", 10);

    timer_ = create_wall_timer(std::chrono::seconds(1), [&]() {
        diagnostic_msgs::msg::DiagnosticStatus diagnostic = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic.level = diagnostic_status_;
        diagnostic.name = "ManipulatorDummy";
        diagnostic.hardware_id = "3";

        diagnostic_status_publisher_->publish(diagnostic);
    });
}

rclcpp_action::GoalResponse ManipulatorDummy::handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveArm::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal request position (x=%.1f, y=%.1f, z=%.1f)", (float)goal->pose.pose.position.x, (float)goal->pose.pose.position.y, (float)goal->pose.pose.position.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulatorDummy::handle_cancel_(const std::shared_ptr<GoalHandleMoveArm>)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulatorDummy::handle_accepted_(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
{
    std::thread{[&](const std::shared_ptr<GoalHandleMoveArm> goal_handle) {
                    RCLCPP_INFO(get_logger(), "Executing goal");

                    std::shared_ptr<MoveArm::Feedback> feedback = std::make_shared<MoveArm::Feedback>();
                    std::shared_ptr<MoveArm::Result> result = std::make_shared<MoveArm::Result>();

                    const std::shared_ptr<const MoveArm::Goal> goal = goal_handle->get_goal();

                    Point goal_position = Point((float)goal->pose.pose.position.x, (float)goal->pose.pose.position.y);

                    double start_time = now().seconds();
                    double last_feedback_time = start_time;
                    float start_distance = current_position_.distance(goal_position);

                    //ONLY FOR XY PLANE: increment unitl given accuracy to goal point is reached
                    //Z IS IGNORED: Point.h has to be adapted to 3D coordinates
                    //GRIPPER IS IGNORED: maybe create separate gripper dummy
                    while (current_position_.distance(goal_position) >= accuracy_)
                    {
                        // Check if there is a cancel request
                        if (goal_handle->is_canceling())
                        {
                            goal_handle->canceled(result);
                            RCLCPP_INFO(get_logger(), "Goal Canceled");
                            return;
                        }

                        if (!rclcpp::ok())
                        {
                            return;
                        }

                        current_position_ = current_position_.increment(goal_position, accuracy_);

                        // Publish feedback every second
                        if ((now().seconds() - last_feedback_time) >= 1)
                        {
                            feedback->movement_time.sec = now().seconds() - start_time;
                            feedback->current_pose.pose.position.x = current_position_.x;
                            feedback->current_pose.pose.position.y = current_position_.y;
                            // Z is set directly to goal value
                            feedback->current_pose.pose.position.z = goal->pose.pose.position.z;
                            feedback->progress = 1 - ((current_position_.distance(goal_position)) / start_distance);

                            goal_handle->publish_feedback(feedback);
                            RCLCPP_INFO(get_logger(), "Publish Feedback");

                            last_feedback_time = now().seconds();
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds((int)((accuracy_ / velocity_) * 1000)));
                    }

                    result->success = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(get_logger(), "Goal Succeeded");
                },
                goal_handle}
        .detach();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManipulatorDummy>());
    rclcpp::shutdown();

    return 0;
}