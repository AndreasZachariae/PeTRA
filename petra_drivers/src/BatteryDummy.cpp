#include <petra_drivers/BatteryDummy.h>

BatteryDummy::BatteryDummy() : Node("BatteryDummy")
{
    set_battery_subscription_ = create_subscription<std_msgs::msg::Float32>("SetBattery", 10, std::bind(&BatteryDummy::set_battery_callback_, this, std::placeholders::_1));

    battery_state_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>("BatteryState", 10);

    diagnostic_status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("DiagnosticStatus", 10);

    charge_battery_server_ = rclcpp_action::create_server<ChargeBattery>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        "ChargeBattery",
        std::bind(&BatteryDummy::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&BatteryDummy::handle_cancel_, this, std::placeholders::_1),
        std::bind(&BatteryDummy::handle_accepted_, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&BatteryDummy::timer_callback_, this));
}

void BatteryDummy::timer_callback_()
{
    auto state = sensor_msgs::msg::BatteryState();
    state.percentage = battery_percentage_;

    auto diagnostic = diagnostic_msgs::msg::DiagnosticStatus();
    diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostic.name = "BatteryDummy";
    diagnostic.hardware_id = "2";

    //RCLCPP_INFO(get_logger(), "Publishing BatteryState and DiagnosticStatus");
    battery_state_publisher_->publish(state);
    diagnostic_status_publisher_->publish(diagnostic);
}

rclcpp_action::GoalResponse BatteryDummy::handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ChargeBattery::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received charging request to %.1f%%", (goal->goal_percentage * 100));
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BatteryDummy::handle_cancel_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BatteryDummy::execute_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle)
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

void BatteryDummy::handle_accepted_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle)
{
    std::thread{std::bind(&BatteryDummy::execute_, this, std::placeholders::_1), goal_handle}.detach();
}

void BatteryDummy::set_battery_callback_(const std_msgs::msg::Float32::SharedPtr msg)
{
    //set percentage value from 0 to 1
    if ((msg->data >= 0) && (msg->data <= 1))
    {
        battery_percentage_ = msg->data;
    }
    //if value is negative, subtract from current percentage, max -1
    else if ((msg->data < 0) && (msg->data >= -1))
    {
        battery_percentage_ += msg->data;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Invalid battery percentage!");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryDummy>());
    rclcpp::shutdown();
    return 0;
}