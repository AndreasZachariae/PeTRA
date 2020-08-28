#include <petra_central_control/actions/ArmMovement.h>

void ArmMovement::onSend(MoveArm::Goal &goal)
{
    // Check if Optional is valid. If not, throw its error
    BT::Optional<float> x_input = getInput<float>("x");
    BT::Optional<float> y_input = getInput<float>("y");
    BT::Optional<float> z_input = getInput<float>("z");
    BT::Optional<float> gripper_input = getInput<float>("gripper_position");

    if (!x_input.has_value())
    {
        throw BT::RuntimeError("missing required input [x]: ", x_input.error());
    }
    if (!y_input.has_value())
    {
        throw BT::RuntimeError("missing required input [y]: ", y_input.error());
    }
    if (!z_input.has_value())
    {
        throw BT::RuntimeError("missing required input [z]: ", z_input.error());
    }
    if (!gripper_input.has_value())
    {
        throw BT::RuntimeError("missing required input [gripper_position]: ", gripper_input.error());
    }

    goal.pose.pose.position.x = x_input.value();
    goal.pose.pose.position.y = y_input.value();
    goal.pose.pose.position.z = z_input.value();
    goal.gripper_position = gripper_input.value();

    goal.pose.pose.orientation.w = 1; //hard-coded orientation (x=0, y=0, z=0, w=1)
    goal.pose.header.stamp = get_node_handle()->now();

    log("Goal: (x=" + ftos(goal.pose.pose.position.x) + ", y=" + ftos(goal.pose.pose.position.y) + ", z=" + ftos(goal.pose.pose.position.z) + ")");
}

void ArmMovement::onFeedback(const std::shared_ptr<const MoveArm::Feedback> feedback)
{
    log("Position: (x=" + ftos((float)feedback->current_pose.pose.position.x) +
        ", y=" + ftos((float)feedback->current_pose.pose.position.y) +
        ", z=" + ftos((float)feedback->current_pose.pose.position.z) +
        "), Progress: " + ftos((float)(feedback->progress * 100)) +
        "%, Time: " + std::to_string(feedback->movement_time.sec) + "s");
}

void ArmMovement::onResult(const rclcpp_action::ClientGoalHandle<MoveArm>::WrappedResult &, const MoveArm::Goal &goal)
{
    log("Goal reached! (x=" + ftos(goal.pose.pose.position.x) +
        ", y=" + ftos(goal.pose.pose.position.y) +
        ", z=" + ftos(goal.pose.pose.position.z) +
        "), Total time: " + std::to_string((int)(get_node_handle()->now().seconds() - -goal.pose.header.stamp.sec)) + "s");
}