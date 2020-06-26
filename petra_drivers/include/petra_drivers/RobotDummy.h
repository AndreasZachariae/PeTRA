/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>

#include <petra_core/Point.h>
#include <petra_core/action/charge_battery.hpp>

using ChargeBattery = petra_core::action::ChargeBattery;
using GoalHandleChargeBattery = rclcpp_action::ServerGoalHandle<ChargeBattery>;

class RobotDummy : public rclcpp::Node
{
public:
    RobotDummy();

private:
    rclcpp_action::Server<ChargeBattery>::SharedPtr charge_battery_server_;

    rclcpp_action::GoalResponse handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ChargeBattery::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle);
    
    void handle_accepted_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle);
    void execute_(const std::shared_ptr<GoalHandleChargeBattery> goal_handle);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    void goal_callback_(geometry_msgs::msg::PoseStamped::UniquePtr goal);
    void stop_callback_(const std_msgs::msg::Empty::SharedPtr msg);

    void simulate_movement_(Point goal);

    bool stop_flipflop_ = false;
    Point current_position_ = Point(0, 0);

    // accuracy in m
    float accuracy_ = 0.1;
    // velocity in m/s
    float velocity_ = 1;

    float battery_percentage_ = 0.5;
};