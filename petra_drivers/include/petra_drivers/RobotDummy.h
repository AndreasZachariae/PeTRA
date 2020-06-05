/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>

#include <petra_core/Point.h>

class RobotDummy : public rclcpp::Node
{
public:
    RobotDummy();

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    void simulate_movement_(Point goal);

    void goal_callback_(geometry_msgs::msg::PoseStamped::UniquePtr goal);
    void stop_callback_(const std_msgs::msg::Empty::SharedPtr msg);

    bool stop_flipflop_ = false;
    Point current_position_ = Point(0, 0);

    // accuracy in m
    float accuracy_ = 0.1;
    //velocity in m/s
    float velocity_ = 1;
};