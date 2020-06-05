#pragma once

#include <petra_central_control/default.h>
#include <petra_central_control/SkillState.h>
#include <petra_central_control/Progress.h>
#include <thread>

class Skill : public Component
{
public:
    Skill(std::string name) : Component(name) {}
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

    virtual void init_() {}
    virtual void start_() {}
    virtual void spin_() {}
    virtual void stop_() {}
    virtual SkillState handle_error_() { return SkillState::error; }

private:
    SkillState state_ = SkillState::uninitialized;

    std::thread thread_;

    void set_state_(SkillState state);
};