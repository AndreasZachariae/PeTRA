#pragma once

#include <petra_central_control/default.h>

class SkillState
{
public:
    enum Value
    {
        uninitialized = 0,
        initialized = 1,
        active = 2,
        stopped = 3,
        succeeded = 4,
        failed = 5
    };

    SkillState(Value value) : value_(value) {}
    SkillState() : SkillState(uninitialized) {}

    operator Value() const { return value_; }
    explicit operator bool() = delete;

    std::string to_string()
    {
        switch (value_)
        {
        case SkillState::uninitialized:
            return "Uninitialized";
        case SkillState::initialized:
            return "Initialized";
        case SkillState::active:
            return "Active";
        case SkillState::stopped:
            return "Stopped";
        case SkillState::succeeded:
            return "Succeeded";
        case SkillState::failed:
            return "Failed";

        default:
            return "";
        }
    }

private:
    Value value_;
};