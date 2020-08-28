#include <petra_central_control/actions/BaseMovement.h>

void BaseMovement::onSend(NavigateToPose::Goal &goal)
{
    // Check if Optional is valid. If not, throw its error
    BT::Optional<float> x_input = getInput<float>("x");
    BT::Optional<float> y_input = getInput<float>("y");

    if (!x_input.has_value())
    {
        throw BT::RuntimeError("missing required input [x]: ", x_input.error());
    }

    if (!y_input.has_value())
    {
        throw BT::RuntimeError("missing required input [y]: ", y_input.error());
    }

    goal.pose.pose.position.x = x_input.value();
    goal.pose.pose.position.y = y_input.value();

    goal.pose.pose.position.z = 0;    //z-value not neccessary
    goal.pose.pose.orientation.w = 1; //hard-coded orientation (x=0, y=0, z=0, w=1)
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = get_node_handle()->now();

    log("Goal: (x=" + ftos(goal.pose.pose.position.x) + ", y=" + ftos(goal.pose.pose.position.y) + ")");
}

void BaseMovement::onFeedback(const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    log("Current position: (x=" + ftos((float)feedback->current_pose.pose.position.x) +
        ", y=" + ftos((float)feedback->current_pose.pose.position.y) +
        "), Time elapsed: " + std::to_string(feedback->navigation_time.sec) + "s");
}

void BaseMovement::onResult(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &, const NavigateToPose::Goal &goal)
{
    log("Goal reached! (x=" + ftos(goal.pose.pose.position.x) +
        ", y=" + ftos(goal.pose.pose.position.y) +
        "), Total time: " + std::to_string((int)get_node_handle()->now().seconds() - goal.pose.header.stamp.sec) + "s");
}