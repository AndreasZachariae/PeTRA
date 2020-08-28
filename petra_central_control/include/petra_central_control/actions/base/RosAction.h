/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Base for all BT-Actions with a ROS2-Action client
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <memory>

#include <rclcpp_action/rclcpp_action.hpp>

#include <petra_central_control/actions/base/Action.h>

/**
 * ROSAction provides an interface to use a ROS2-Action client in a BT-Action.
 * Template type has to be a (custom) .action type generated by ROS2.
 */
template <typename ActionT>
class RosAction : public Action
{
public:
    using GoalT = typename ActionT::Goal;
    using FeedbackT = typename ActionT::Feedback;
    using ResultT = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
    using OptionsT = typename rclcpp_action::Client<ActionT>::SendGoalOptions;

    RosAction(const std::string &name, const BT::NodeConfiguration &config) : Action(name, config)
    {
        progress_.steps = 3;
    }

    /**
     * Override and return unique ROS2-Action name. 
     * This has to be the same as the ROS2-Action server name.
     */
    virtual std::string actionServerName() = 0;

    /**
     * This is called before the goal is sent asynchronously to the ROS2-Action server.
     * @param goal Goal-section data of the .action file
     */
    virtual void onSend(GoalT &goal) = 0;

    /**
     * This is called if feedback is recieved from the ROS2-Action server. 
     * Override only if neccessary.
     * @param feedback Feedback-section data of the .action file
     */
    virtual void onFeedback(const std::shared_ptr<const FeedbackT>) {}

    /**
     * This is called if the ROS2-Action server has successfully finished. 
     * @param result Result-section data of the .action file
     * @param goal Goal-section data of the .action file
     */
    virtual void onResult(const ResultT &result, const GoalT &goal) = 0;

private:
    Progress progress_;

    std::shared_ptr<rclcpp_action::Client<ActionT>> client_;
    std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>>> future_;
    GoalT goal_;
    ResultT result_;

    BT::NodeStatus onStart() override
    {
        log("Connecting to action server: " + actionServerName());

        progress_.current_step = 0;

        client_ = rclcpp_action::create_client<ActionT>(
            get_node_handle()->get_node_base_interface(),
            get_node_handle()->get_node_graph_interface(),
            get_node_handle()->get_node_logging_interface(),
            get_node_handle()->get_node_waitables_interface(),
            actionServerName());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        switch (progress_.current_step)
        {
        case Progress::FAIL_STEP:
            return BT::NodeStatus::FAILURE;
        case 0:
            if (client_->action_server_is_ready())
            {
                send_();
            }
            break;
        case 3:
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        try
        {
            client_->async_cancel_goal(future_.get());
        }
        catch (const std::future_error &e)
        {
            log("Canceled before goal was sent.");
        }
    }

    void send_()
    {
        OptionsT options = OptionsT();

        options.goal_response_callback = [&](std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>>>) {
            if (!future_.get())
            {
                progress_.set_fail("Goal was rejected!");
            }
            else
            {
                progress_.next_step("Goal accepted.");
            }
        };

        options.feedback_callback = [&](std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>>, const std::shared_ptr<const FeedbackT> feedback) {
            onFeedback(feedback);
        };

        options.result_callback = [&](const ResultT &result) {
            result_ = result;

            if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                progress_.set_fail("Goal canceled!");
            }
            else if (result.code == rclcpp_action::ResultCode::ABORTED ||
                     result.code == rclcpp_action::ResultCode::UNKNOWN)
            {
                progress_.set_fail("Goal failed!");
            }
            else
            {
                onResult(result_, goal_);

                progress_.next_step("Received result.");
            }
        };

        goal_ = GoalT();

        onSend(goal_);

        future_ = client_->async_send_goal(goal_, options);

        progress_.next_step("Goal sent.");
    }
};