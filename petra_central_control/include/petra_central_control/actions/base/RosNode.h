#pragma once

#include <petra_central_control/default.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <petra_central_control/actions/base/Progress.h>

class RosNode : public Component
{
public:
    static rclcpp::Node::SharedPtr get_node_handle();

    static void init(int argc, char **argv) { rclcpp::init(argc, argv); }
    static void init() { init(0, nullptr); }

    static void spin_some() { rclcpp::spin_some(get_node_handle()); }

    static void shutdown() { rclcpp::shutdown(); }

    RosNode(const std::string &name);
    ~RosNode();

protected:
    Progress progress_;

private:
    static rclcpp::Node::SharedPtr node_handle_;

    static uint instance_count_;
};