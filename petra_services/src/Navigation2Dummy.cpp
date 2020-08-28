#include <petra_services/Navigation2Dummy.h>

Navigation2Dummy::Navigation2Dummy() : Node("Navigation2Dummy")
{
    navigate_to_pose_server_ = rclcpp_action::create_server<NavigateToPose>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        "NavigateToPose",
        std::bind(&Navigation2Dummy::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Navigation2Dummy::handle_cancel_, this, std::placeholders::_1),
        std::bind(&Navigation2Dummy::handle_accepted_, this, std::placeholders::_1));

    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("move_base_simple/goal", 10);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10,
                                                                          [&](nav_msgs::msg::Odometry::UniquePtr odom) {
                                                                              current_position_.x = (float)odom->pose.pose.position.x;
                                                                              current_position_.y = (float)odom->pose.pose.position.y;
                                                                          });
}

rclcpp_action::GoalResponse Navigation2Dummy::handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const NavigateToPose::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal request position (x=%i, y=%i)", (int)goal->pose.pose.position.x, (int)goal->pose.pose.position.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Navigation2Dummy::handle_cancel_(const std::shared_ptr<GoalHandleNavigateToPose>)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Navigation2Dummy::handle_accepted_(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
    std::thread{[&](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
                    RCLCPP_INFO(get_logger(), "Executing goal");

                    std::shared_ptr<NavigateToPose::Feedback> feedback = std::make_shared<NavigateToPose::Feedback>();
                    std::shared_ptr<NavigateToPose::Result> result = std::make_shared<NavigateToPose::Result>();

                    const std::shared_ptr<const NavigateToPose::Goal> goal = goal_handle->get_goal();

                    RCLCPP_INFO(get_logger(), "Publishing: move_base_simple/goal");
                    goal_publisher_->publish(goal->pose);

                    //current position from topic odometry_callback, default (x=0,y=0)
                    Point goal_position = Point((float)goal->pose.pose.position.x, (float)goal->pose.pose.position.y);

                    double start_time = now().seconds();
                    double last_feedback_time = start_time;

                    //wait for given accuracy to goal_position
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

                        // Publish feedback every second
                        if ((now().seconds() - last_feedback_time) >= 1)
                        {
                            feedback->navigation_time.sec = now().seconds() - start_time;
                            feedback->current_pose.pose.position.x = current_position_.x;
                            feedback->current_pose.pose.position.y = current_position_.y;

                            goal_handle->publish_feedback(feedback);
                            RCLCPP_INFO(get_logger(), "Publish Feedback");

                            last_feedback_time = now().seconds();
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }

                    goal_handle->succeed(result);
                    RCLCPP_INFO(get_logger(), "Goal Succeeded");
                },
                goal_handle}
        .detach();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Navigation2Dummy>());
    rclcpp::shutdown();

    return 0;
}