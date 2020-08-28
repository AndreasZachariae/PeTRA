#include <petra_services/PatientMonitoringDummy.h>

PatientMonitoringDummy::PatientMonitoringDummy() : Node("PatientMonitoringDummy")
{
    condition_publisher_ = create_publisher<petra_core::msg::PatientCondition>("PatientCondition", 10);

    stop_publisher_ = create_publisher<std_msgs::msg::Empty>("Stop", 10);

    emergency_publisher_ = create_publisher<petra_core::msg::EmergencyStop>("EmergencyStop", 10);

    set_condition_subscription_ = create_subscription<std_msgs::msg::String>("SetCondition", 10, [&](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "good")
        {
            auto msg = petra_core::msg::PatientCondition();
            msg.condition = petra_core::msg::PatientCondition::GOOD;
            RCLCPP_INFO(get_logger(), "Publishing condition 'good'");

            condition_publisher_->publish(msg);
        }
        else if (msg->data == "bad")
        {
            auto msg = petra_core::msg::PatientCondition();
            msg.condition = petra_core::msg::PatientCondition::BAD;
            RCLCPP_INFO(get_logger(), "Publishing condition 'bad' and /Stop");

            condition_publisher_->publish(msg);
            //stop_publisher_->publish(std_msgs::msg::Empty());
        }
        else if (msg->data == "emergency")
        {
            auto msg = petra_core::msg::PatientCondition();
            msg.condition = petra_core::msg::PatientCondition::EMERGENCY;

            auto emergency = petra_core::msg::EmergencyStop();
            emergency.urgency = petra_core::msg::EmergencyStop::HIGH;
            emergency.cause = "From PatientMonitoringDummy";
            emergency.software_stop = true;

            RCLCPP_INFO(get_logger(), "Publishing condition 'emergency' and /Stop and /EmergencyStop");

            condition_publisher_->publish(msg);
            stop_publisher_->publish(std_msgs::msg::Empty());
            emergency_publisher_->publish(emergency);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Invalid input condition");
        }
    });
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatientMonitoringDummy>());
    rclcpp::shutdown();

    return 0;
}