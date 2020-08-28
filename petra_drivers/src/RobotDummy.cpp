#include <petra_drivers/RobotDummy.h>

RobotDummy::RobotDummy() : Node("RobotDummy")
{
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>("move_base_simple/goal", 10, [&](geometry_msgs::msg::PoseStamped::UniquePtr goal) {
        RCLCPP_INFO(get_logger(), "Goal position (x=%.1f, y=%.1f)", goal->pose.position.x, goal->pose.position.y);
        RCLCPP_INFO(get_logger(), "Start position (x=%.1f, y=%.1f)", current_position_.x, current_position_.y);

        Point goal_point = Point((float)goal->pose.position.x, (float)goal->pose.position.y);

        //start in a new thread damit callback nicht blockiert
        std::thread{std::bind(&RobotDummy::simulate_movement_, this, std::placeholders::_1), goal_point}.detach();
    });

    stop_subscription_ = create_subscription<std_msgs::msg::Empty>("Stop", 10, [&](const std_msgs::msg::Empty::SharedPtr) {
        stop_recieved_ = true;
        RCLCPP_WARN(get_logger(), "Stop recieved, resetting...");
    });

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    diagnostic_status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("DiagnosticStatus", 10);

    set_battery_publisher_ = create_publisher<std_msgs::msg::Float32>("SetBattery", 10);

    timer_ = create_wall_timer(std::chrono::seconds(1), [&]() {
        diagnostic_msgs::msg::DiagnosticStatus diagnostic = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic.level = diagnostic_status_;
        diagnostic.name = "RobotDummy";
        diagnostic.hardware_id = "1";

        diagnostic_status_publisher_->publish(diagnostic);
    });
}

void RobotDummy::simulate_movement_(Point goal)
{
    stop_recieved_ = false;

    Point start(current_position_.x, current_position_.y);

    nav_msgs::msg::Odometry odom = nav_msgs::msg::Odometry();

    while ((current_position_.distance(goal) >= accuracy_) && rclcpp::ok() && !stop_recieved_)
    {
        current_position_ = current_position_.increment(goal, accuracy_);

        odom.pose.pose.position.x = current_position_.x;
        odom.pose.pose.position.y = current_position_.y;

        odometry_publisher_->publish(odom);
        set_battery_publisher_->publish(std_msgs::msg::Float32().set__data(-0.001));

        std::this_thread::sleep_for(std::chrono::milliseconds((int)((accuracy_ / velocity_) * 1000)));
    }

    RCLCPP_INFO(get_logger(), "Current position: (x=%.1f, y=%.1f) accuracy=%.1fm", current_position_.x, current_position_.y, accuracy_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDummy>());
    rclcpp::shutdown();

    return 0;
}