#include "rclcpp/rclcpp.hpp"

class MinimalNodeRunLoop
{
private:
    std::shared_ptr<rclcpp::Node> node_handle_;

public:
    MinimalNodeRunLoop();
    ~MinimalNodeRunLoop();
    void run();
};

MinimalNodeRunLoop::MinimalNodeRunLoop()
{
    node_handle_ = rclcpp::Node::make_shared("MinimalNodeRunLoop");
}

MinimalNodeRunLoop::~MinimalNodeRunLoop()
{
    rclcpp::shutdown();
}

void MinimalNodeRunLoop::run()
{
    while (rclcpp::ok())
    {
        //publish etc...
        rclcpp::spin_some(node_handle_);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    MinimalNodeRunLoop node;
    node.run();
    return 0;
}