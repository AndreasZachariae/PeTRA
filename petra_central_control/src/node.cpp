/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : main-function and start of the behavior tree
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#include <behaviortree_cpp_v3/bt_factory.h>

#include <petra_central_control/actions/base/RosNode.h>

#include <petra_central_control/controls/SwitchNode.h>
#include <petra_central_control/actions/ParameterRequest.h>
#include <petra_central_control/actions/DevicePairing.h>
#include <petra_central_control/actions/BaseMovement.h>
#include <petra_central_control/actions/ArmMovement.h>
#include <petra_central_control/actions/BatteryCharging.h>
#include <petra_central_control/conditions/CheckStop.h>
#include <petra_central_control/conditions/CheckBattery.h>
#include <petra_central_control/conditions/CheckDiagnosticStatus.h>
#include <petra_central_control/conditions/CheckPatientCondition.h>
#include <petra_central_control/conditions/CheckBlackboard.h>

BT::Tree create_tree(const std::string &path)
{
    BT::BehaviorTreeFactory factory;
    //factory.registerNodeType<BT::BlackboardPreconditionNode<std::string>>("CheckBlackboard");
    factory.registerNodeType<BT::SwitchNode<8>>("BehaviorSwitch");
    factory.registerNodeType<BT::AlwaysFailureNode>("AlwaysFailureNode");

    factory.registerNodeType<CheckStop>("CheckStop");
    factory.registerNodeType<CheckBattery>("CheckBattery");
    factory.registerNodeType<CheckDiagnosticStatus>("CheckDiagnosticStatus");
    factory.registerNodeType<CheckPatientCondition>("CheckPatientCondition");
    factory.registerNodeType<CheckBlackboard>("CheckBlackboard");

    factory.registerNodeType<ParameterRequest<bool>>("BoolParameterRequest");
    factory.registerNodeType<ParameterRequest<int>>("IntParameterRequest");
    factory.registerNodeType<ParameterRequest<float>>("FloatParameterRequest");
    factory.registerNodeType<ParameterRequest<std::string>>("StringParameterRequest");
    factory.registerNodeType<DevicePairing>("DevicePairing");
    factory.registerNodeType<BaseMovement>("BaseMovement");
    factory.registerNodeType<ArmMovement>("ArmMovement");
    factory.registerNodeType<BatteryCharging>("BatteryCharging");

    return factory.createTreeFromFile(path);
}

BT::NodeStatus run_tree(BT::Tree &tree)
{
    BT::NodeStatus status;
    BT::NodeStatus last_status;

    do
    {
        RosNode::spin_some();

        status = tree.root_node->executeTick();

        if (status != last_status)
        {
            switch (status)
            {
            case BT::NodeStatus::IDLE:
                std::cout << "[MainTree] Status: IDLE" << std::endl;
                break;
            case BT::NodeStatus::RUNNING:
                std::cout << "[MainTree] Status: RUNNING" << std::endl;
                break;
            case BT::NodeStatus::SUCCESS:
                std::cout << "[MainTree] Status: SUCCESS" << std::endl;
                break;
            case BT::NodeStatus::FAILURE:
                std::cout << "[MainTree] Status: FAILURE" << std::endl;
                break;
            }

            last_status = status;
        }

    } while (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING);

    return status;
}

int main(int, char **)
{
    std::cout << std::string(getenv("HOME")) + "/petra_ws/src/petra_central_control/behaviors/MainTree.xml" << std::endl;
    BT::Tree tree = create_tree(std::string(getenv("HOME")) + "/petra_ws/src/petra_central_control/behaviors/MainTree.xml");

    run_tree(tree);

    return 0;
}