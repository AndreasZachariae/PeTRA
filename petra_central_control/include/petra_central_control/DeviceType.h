#pragma once

#include <petra_central_control/default.h>
#include <petra_core/action/pair_device.hpp>

class DeviceType
{
public:
    enum Value
    {
        wheelchair = petra_core::action::PairDevice::Goal::WHEELCHAIR,
        rollator = petra_core::action::PairDevice::Goal::ROLLATOR,
        hospitalbed = petra_core::action::PairDevice::Goal::HOSPITALBED,
        chargingstation = petra_core::action::PairDevice::Goal::CHARGINGSTATION
    };

    DeviceType(Value value) : value_(value) {}
    DeviceType() : DeviceType(wheelchair) {}

    operator Value() const { return value_; }
    explicit operator bool() = delete;

    std::string to_string()
    {
        switch (value_)
        {
        case DeviceType::wheelchair:
            return "Wheelchair";
        case DeviceType::rollator:
            return "Rollator";
        case DeviceType::hospitalbed:
            return "HospitalBed";
        case DeviceType::chargingstation:
            return "ChargingStation";

        default:
            return "";
        }
    }

private:
    Value value_;
};