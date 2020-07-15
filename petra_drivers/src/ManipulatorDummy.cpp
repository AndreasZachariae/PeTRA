#include <petra_drivers/ManipulatorDummy.h>
#include <math.h>

ManipulatorDummy::ManipulatorDummy(const rclcpp::NodeOptions &options) : Node("ManipulatorDummy", options)
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

    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ManipulatorDummy::timer_callback_, this));
}

void ManipulatorDummy::timer_callback_()
{
    auto diagnostic = diagnostic_msgs::msg::DiagnosticStatus();
    diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostic.name = "ManipulatorDummy";
    diagnostic.hardware_id = "3";

    //RCLCPP_INFO(get_logger(), "Publishing DiagnosticStatus");
    diagnostic_status_publisher_->publish(diagnostic);
}

rclcpp_action::GoalResponse ManipulatorDummy::handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveArm::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal request position (x=%.1f, y=%.1f, z=%.1f)", (float)goal->pose.position.x, (float)goal->pose.position.y, (float)goal->pose.position.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulatorDummy::handle_cancel_(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulatorDummy::execute_(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing goal");

    auto feedback = std::make_shared<MoveArm::Feedback>();
    auto result = std::make_shared<MoveArm::Result>();

    const auto goal = goal_handle->get_goal();

    Point goal_position = Point((float)goal->pose.position.x, (float)goal->pose.position.y, (float)goal->pose.position.z);

    auto start_time = now().seconds();
    auto last_feedback_time = start_time;
    float start_distance = current_position_.distance(goal_position);

    //NUR IN XY EBENE: warte bis vorgegebene Genauigkeit zum Zielpunkt erreicht ist
    //Z wird ingnoriert
    while ((current_position_.distance(goal_position) >= (accuracy_ + fabs(goal_position.z - current_position_.z))) && rclcpp::ok())
    {
        // Check if there is a cancel request
        if (goal_handle->is_canceling())
        {
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal Canceled");
            return;
        }

        current_position_ = current_position_.increment(goal_position, accuracy_);

        std::this_thread::sleep_for(std::chrono::milliseconds((int)((accuracy_ / velocity_) * 1000)));

        // Publish feedback every second
        if ((now().seconds() - last_feedback_time) >= 1)
        {
            feedback->movement_time.sec = now().seconds() - start_time;
            feedback->current_pose.position.x = current_position_.x;
            feedback->current_pose.position.y = current_position_.y;
            feedback->current_pose.position.z = current_position_.z;
            feedback->progress = 1 - ((current_position_.distance(goal_position)) / start_distance);

            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish Feedback");

            last_feedback_time = now().seconds();
        }
    }
    
    // Check if goal is done
    if (rclcpp::ok())
    {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal Succeeded");
    }
}

void ManipulatorDummy::handle_accepted_(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
{
    std::thread{std::bind(&ManipulatorDummy::execute_, this, std::placeholders::_1), goal_handle}.detach();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManipulatorDummy>());
    rclcpp::shutdown();
    return 0;
}