#include <petra_central_control/actions/base/RosNode.h>

rclcpp::Node::SharedPtr RosNode::node_handle_ = nullptr;

uint RosNode::instance_count_ = 0;

rclcpp::Node::SharedPtr RosNode::get_node_handle()
{
    if (RosNode::node_handle_ == nullptr)
    {
        RosNode::node_handle_ = std::make_shared<rclcpp::Node>("PeTRACentralControl");
    }

    return RosNode::node_handle_;
}

RosNode::RosNode(const std::string &name) : Component(name)
{
    if (instance_count_++ == 0 && !rclcpp::is_initialized())
    {
        init();
    }
}

RosNode::~RosNode()
{
    if (--instance_count_ == 0)
    {
        shutdown();
    }
}