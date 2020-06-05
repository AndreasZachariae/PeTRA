/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#include <petra_central_control/CentralControlUnit.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    CentralControlUnit ccu(argc, argv);
    ccu.run();

    rclcpp::shutdown();

    return 0;
}