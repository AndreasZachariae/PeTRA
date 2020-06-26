#include <petra_drivers/RobotDummy.h>

RobotDummy::RobotDummy() : Node("RobotDummy")
{
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>("move_base_simple/goal", 10, std::bind(&RobotDummy::goal_callback_, this, std::placeholders::_1));
    stop_subscription_ = create_subscription<std_msgs::msg::Empty>("Stop", 10, std::bind(&RobotDummy::stop_callback_, this, std::placeholders::_1));
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    charge_battery_server_ = rclcpp_action::create_server<ChargeBattery>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        "ChargeBattery",
        std::bind(&RobotDummy::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&RobotDummy::handle_cancel_, this, std::placeholders::_1),
        std::bind(&RobotDummy::handle_accepted_, this, std::placeholders::_1));
}

void RobotDummy::simulate_movement_(Point goal)
{
    bool stop = !stop_flipflop_;

    Point start(current_position_.x, current_position_.y);

    auto odom = nav_msgs::msg::Odometry();

    while ((current_position_.distance(goal) >= accuracy_) && rclcpp::ok() && (stop_flipflop_ != stop))
    {
        current_position_ = current_position_.increment(goal, accuracy_);

        odom.pose.pose.position.x = current_position_.x;
        odom.pose.pose.position.y = current_position_.y;

        odometry_publisher_->publish(odom);

        battery_percentage_ -= 0.001;

        std::this_thread::sleep_for(std::chrono::milliseconds((int)((accuracy_ / velocity_) * 1000)));
    }

    RCLCPP_INFO(get_logger(), "Current position: (x=%.1f, y=%.1f) accuracy=%.1fm", current_position_.x, current_position_.y, accuracy_);
}

void RobotDummy::goal_callback_(geometry_msgs::msg::PoseStamped::UniquePtr goal)
{
    RCLCPP_INFO(get_logger(), "Goal position (x=%.1f, y=%.1f)", goal->pose.position.x, goal->pose.position.y);
    RCLCPP_INFO(get_logger(), "Start position (x=%.1f, y=%.1f)", current_position_.x, current_position_.y);

    Point goal_point = Point((float)goal->pose.position.x, (float)goal->pose.position.y);

    //start in a new thread damit callback nicht blockiert
    std::thread{std::bind(&RobotDummy::simulate_movement_, this, std::placeholders::_1), goal_point}.detach();
}

void RobotDummy::stop_callback_(const std_msgs::msg::Empty::SharedPtr msg)
{
    stop_flipflop_ = !stop_flipflop_;
    RCLCPP_WARN(get_logger(), "Stop recieved, resetting...");
    (void)msg;
}

rclcpp_action::GoalResponse RobotDummy::handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ChargeBattery::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received charging request to %.1f%%", (goal->goal_percentage * 100));
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotDummy::handle_cancel_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotDummy::execute_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing goal");

    auto feedback = std::make_shared<ChargeBattery::Feedback>();
    auto result = std::make_shared<ChargeBattery::Result>();

    const auto goal = goal_handle->get_goal();

    //charge 10% per second
    while ((battery_percentage_ < goal->goal_percentage) && rclcpp::ok())
    {
        battery_percentage_ += 0.1;
        feedback->current_state.percentage = battery_percentage_;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "Publish Feedback");
    }

    if (rclcpp::ok())
    {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Charging succeeded");
    }
}

void RobotDummy::handle_accepted_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle)
{
    std::thread{std::bind(&RobotDummy::execute_, this, std::placeholders::_1), goal_handle}.detach();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDummy>());
    rclcpp::shutdown();
    return 0;
}