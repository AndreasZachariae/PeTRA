#pragma once

#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <petra_central_control/default.h>
#include <petra_central_control/SkillState.h>
#include <petra_central_control/Progress.h>

#include <std_msgs/msg/empty.hpp>

class Skill : public Component
{
public:
    Skill(std::shared_ptr<rclcpp::Node> node_handle, std::string name, bool background = false);
    ~Skill() {}

    void init();
    void start();
    void spin();
    void stop();

    SkillState get_state() { return state_; }
    Progress get_progress() { return progress_; }

protected:
    Progress progress_;

    void finish_();
    void error_();
    void next_step_(std::string msg = "...");
    void set_step_(int step);

    virtual void init_() {}
    virtual void start_() {}
    virtual void spin_() {}
    virtual void stop_() {}
    virtual SkillState handle_error_() { return SkillState::error; }

private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;

    SkillState state_ = SkillState::uninitialized;

    std::thread thread_;
    std::shared_ptr<rclcpp::Node> node_handle_;

    void set_state_(SkillState state);
};