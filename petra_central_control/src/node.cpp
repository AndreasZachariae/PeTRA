/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#include <behaviortree_cpp_v3/bt_factory.h>

#include <petra_central_control/actions/SkillSelection.h>
#include <petra_central_control/actions/base/RosNode.h>

#include <petra_central_control/controls/SwitchNode.h>
#include <petra_central_control/actions/SkillSelection.h>
#include <petra_central_control/actions/DevicePairing.h>
#include <petra_central_control/actions/BaseMovement.h>
#include <petra_central_control/actions/ArmMovement.h>
#include <petra_central_control/actions/BatteryCharging.h>
#include <petra_central_control/actions/ConfirmDialog.h>
#include <petra_central_control/conditions/CheckStop.h>
#include <petra_central_control/conditions/CheckBattery.h>
#include <petra_central_control/conditions/CheckDiagnosticStatus.h>
#include <petra_central_control/conditions/CheckPatientCondition.h>
#include <petra_central_control/conditions/CheckBlackboard.h>

#include <petra_central_control/actions/BaseMovementRequestGoal.h>
#include <petra_central_control/actions/BaseMovementSendGoal.h>

static const char *xml_text = R"(

 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <ReactiveSequence>
            <CheckStop/>
            <CheckBattery/>
            <CheckDiagnosticStatus/>
            <CheckPatientCondition/>
            <Sequence>
                <Action ID="SkillSelection" selected_action="{selected_action}" />
                <Action ID="DevicePairing"/>
                <Action ID="BaseMovement"/>
                <Action ID="ArmMovement"/>
                <Action ID="BatteryCharging"/>
                <Action ID="ConfirmDialog"/>
            </Sequence>
        </ReactiveSequence>
     </BehaviorTree>
 </root>
 )";

BT::Tree create_tree(const std::string &path)
{
    BT::BehaviorTreeFactory factory;
    //factory.registerNodeType<BT::BlackboardPreconditionNode<std::string>>("CheckBlackboard");
    factory.registerNodeType<BT::SwitchNode<8>>("SkillSwitch");
    factory.registerNodeType<CheckStop>("CheckStop");
    factory.registerNodeType<CheckBattery>("CheckBattery");
    factory.registerNodeType<CheckDiagnosticStatus>("CheckDiagnosticStatus");
    factory.registerNodeType<CheckPatientCondition>("CheckPatientCondition");
    factory.registerNodeType<CheckBlackboard>("CheckBlackboard");

    factory.registerNodeType<BT::DecoratorSubtreeNode>("FreeWalking");
    factory.registerNodeType<BT::DecoratorSubtreeNode>("BaseMovement");
    factory.registerNodeType<BaseMovementRequestGoal>("BaseMovementRequestGoal");
    factory.registerNodeType<BaseMovementSendGoal>("BaseMovementSendGoal");

    factory.registerNodeType<BT::AlwaysFailureNode>("AlwaysFailureNode");
    factory.registerNodeType<SkillSelection>("SkillSelection");
    factory.registerNodeType<DevicePairing>("DevicePairing");
    //factory.registerNodeType<BaseMovement>("BaseMovement");
    factory.registerNodeType<ArmMovement>("ArmMovement");
    factory.registerNodeType<BatteryCharging>("BatteryCharging");
    factory.registerNodeType<ConfirmDialog>("ConfirmDialog");

    //(void)path;
    return factory.createTreeFromFile(path);

    //return factory.createTreeFromText(xml_text);
}

BT::NodeStatus run_tree(BT::Tree &tree)
{
    BT::NodeStatus status;
    BT::NodeStatus logged_status;

    do
    {
        RosNode::spin_some();

        status = tree.root_node->executeTick();

        if (status != logged_status)
        {
            if (status == BT::NodeStatus::RUNNING)
            {
                std::cout << "[MainTree] Status: RUNNING" << std::endl;
                logged_status = BT::NodeStatus::RUNNING;
            }
            else if (status == BT::NodeStatus::IDLE)
            {
                std::cout << "[MainTree] Status: IDLE" << std::endl;
                logged_status = BT::NodeStatus::IDLE;
            }
            else if (status == BT::NodeStatus::FAILURE)
            {
                std::cout << "[MainTree] Status: FAILURE" << std::endl;
                logged_status = BT::NodeStatus::FAILURE;
            }
            else if (status == BT::NodeStatus::SUCCESS)
            {
                std::cout << "[MainTree] Status: SUCCESS" << std::endl;
                logged_status = BT::NodeStatus::SUCCESS;
            }
        }

    } while (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING);

    return status;
}

int main(int argc, char **argv)
{
    BT::Tree tree = create_tree("/home/andreas/petra_ws/src/petra_central_control/behaviors/MainTree.xml");

    BT::NodeStatus status = run_tree(tree);

    if (status == BT::NodeStatus::SUCCESS)
    {
        std::cout << "[Main] Finished successfully" << std::endl;
    }
    else if (status == BT::NodeStatus::FAILURE)
    {
        std::cout << "[Main] Finished with failure" << std::endl;
    }

    return 0;
}