#pragma once

#include <petra_central_control/default.h>

class CCUState
{
public:
    enum Value
    {
        uninitialized = 0,
        initialized = 1,
        active = 2,
        finished = 3
    };

    CCUState(Value value) : value_(value) {}
    CCUState() : CCUState(uninitialized) {}

    operator Value() const { return value_; }
    explicit operator bool() = delete;

    std::string to_string()
    {
        switch (value_)
        {
        case CCUState::uninitialized:
            return "Uninitialized";
        case CCUState::initialized:
            return "Initialized";
        case CCUState::active:
            return "Active";
        case CCUState::finished:
            return "Finished";
        default:
            return "";
        }
    }

private:
    Value value_;
};