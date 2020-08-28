/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Manages the ROS2-Node 
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <petra_central_control/actions/base/Progress.h>

/**
 * Provides the ROS2-Node "PeTRACentralControl". 
 * Base to all classes which use Topics, Services or Actions
 */
class RosNode : public Component
{
public:
    /**
     * Returns the node handle for "PeTRACentralControl". 
     * Creates the node if it doesn't exist yet.
     */
    static rclcpp::Node::SharedPtr get_node_handle();

    /// Initializes the Node in the ROS2-framework
    static void init(int argc, char **argv) { rclcpp::init(argc, argv); }
    static void init() { init(0, nullptr); }

    /**
     * Spins the node_handle in the ROS2-framework. 
     * Executes all callbacks which are currently queued.
     */
    static void spin_some() { rclcpp::spin_some(get_node_handle()); }

    /// Used for clean shutdown of ROS2-Node
    static void shutdown() { rclcpp::shutdown(); }

    RosNode(const std::string &name);
    ~RosNode();

private:
    static rclcpp::Node::SharedPtr node_handle_;

    static uint instance_count_;
};