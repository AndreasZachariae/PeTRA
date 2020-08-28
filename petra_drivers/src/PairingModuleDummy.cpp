#include <petra_drivers/PairingModuleDummy.h>

PairingModuleDummy::PairingModuleDummy() : Node("PairingModuleDummy")
{
    pair_device_server_ = rclcpp_action::create_server<PairDevice>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        "PairDevice",
        std::bind(&PairingModuleDummy::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PairingModuleDummy::handle_cancel_, this, std::placeholders::_1),
        std::bind(&PairingModuleDummy::handle_accepted_, this, std::placeholders::_1));

    diagnostic_status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("DiagnosticStatus", 10);

    timer_ = create_wall_timer(std::chrono::seconds(1), [&]() {
        diagnostic_msgs::msg::DiagnosticStatus diagnostic = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic.level = diagnostic_status_;
        diagnostic.name = "PairingModuleDummy";
        diagnostic.hardware_id = "4";

        diagnostic_status_publisher_->publish(diagnostic);
    });
}

rclcpp_action::GoalResponse PairingModuleDummy::handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PairDevice::Goal> goal)
{
    if ((goal->pairing_mode == PairDevice::Goal::PAIR) && paired_)
    {
        RCLCPP_WARN(get_logger(), "There is already a device paired!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    else if ((goal->pairing_mode == PairDevice::Goal::UNPAIR) && !paired_)
    {
        RCLCPP_WARN(get_logger(), "There is no device to unpair!");
        return rclcpp_action::GoalResponse::REJECT;
    }

    std::string msg;

    if (goal->pairing_mode == PairDevice::Goal::PAIR)
    {
        msg = "Pair";
    }
    else
    {
        msg = "Unpair";
    }

    switch (goal->device_type)
    {
    case PairDevice::Goal::WHEELCHAIR:
        msg = msg + " Wheelchair";
        break;
    case PairDevice::Goal::ROLLATOR:
        msg = msg + " Rollator";
        break;
    case PairDevice::Goal::HOSPITALBED:
        msg = msg + " HospitalBed";
        break;
    case PairDevice::Goal::CHARGINGSTATION:
        msg = msg + " ChargingStation";
        break;

    default:
        break;
    }

    device_ = goal->device_type;

    RCLCPP_INFO(get_logger(), "Received pairing request: %s", msg.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PairingModuleDummy::handle_cancel_(const std::shared_ptr<GoalHandlePairDevice>)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PairingModuleDummy::handle_accepted_(const std::shared_ptr<GoalHandlePairDevice> goal_handle)
{
    std::thread{[&](const std::shared_ptr<GoalHandlePairDevice> goal_handle) {
                    RCLCPP_INFO(get_logger(), "Executing goal");

                    std::shared_ptr<PairDevice::Feedback> feedback = std::make_shared<PairDevice::Feedback>();
                    std::shared_ptr<PairDevice::Result> result = std::make_shared<PairDevice::Result>();

                    const std::shared_ptr<const PairDevice::Goal> goal = goal_handle->get_goal();

                    double start_time = now().seconds();
                    double last_feedback_time = start_time;

                    feedback->progress = 0;

                    while (feedback->progress < 1)
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

                        // Publish feedback every second
                        if ((now().seconds() - last_feedback_time) >= 1)
                        {
                            feedback->progress += 0.2;

                            goal_handle->publish_feedback(feedback);
                            RCLCPP_INFO(get_logger(), "Publish Feedback");

                            last_feedback_time = now().seconds();
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }

                    paired_ = (goal->pairing_mode == PairDevice::Goal::PAIR);

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
    rclcpp::spin(std::make_shared<PairingModuleDummy>());
    rclcpp::shutdown();

    return 0;
}