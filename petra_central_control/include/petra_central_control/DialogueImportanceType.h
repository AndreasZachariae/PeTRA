#pragma once

#include <petra_central_control/default.h>
#include <petra_core/srv/dialogue.hpp>

class DialogueImportanceType
{
public:
    enum Value
    {
        normal = petra_core::srv::Dialogue::Request::NORMAL,
        high = petra_core::srv::Dialogue::Request::HIGH,
        critical = petra_core::srv::Dialogue::Request::CRITICAL
    };

    DialogueImportanceType(Value value) : value_(value) {}
    DialogueImportanceType() : DialogueImportanceType(normal) {}

    operator Value() const { return value_; }
    explicit operator bool() = delete;

    std::string to_string()
    {
        switch (value_)
        {
        case DialogueImportanceType::normal:
            return "Normal";
        case DialogueImportanceType::high:
            return "High";
        case DialogueImportanceType::critical:
            return "Critical";

        default:
            return "";
        }
    }

private:
    Value value_;
};