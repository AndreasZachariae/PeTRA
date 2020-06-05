#include <petra_drivers/RobotDummy.h>

RobotDummy::RobotDummy() : Node("RobotDummy")
{
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>("move_base_simple/goal", 10, std::bind(&RobotDummy::goal_callback_, this, std::placeholders::_1));
    stop_subscription_ = create_subscription<std_msgs::msg::Empty>("Stop", 10, std::bind(&RobotDummy::stop_callback_, this, std::placeholders::_1));
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
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
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDummy>());
    rclcpp::shutdown();
    return 0;
}