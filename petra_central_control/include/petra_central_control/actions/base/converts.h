/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Provides common used conversion functions
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <behaviortree_cpp_v3/basic_types.h>

namespace BT
{
    /**
     * This is called from the BT-framework when reading input ports. 
     * Provides conversion from .xml-String to float.
     */
    template <>
    inline float convertFromString<float>(StringView str)
    {
        return std::stof(str.data());
    }
} // end namespace BT

/**
 * Provides conversion from float to string. 
 * Rounds to two decimal digits and removes trailing zeroes.
 */
inline std::string ftos(float float_value)
{
    std::string float_str = std::to_string(std::roundf(float_value * 100.0) / 100.0);

    float_str.erase(float_str.find_last_not_of('0') + 1, std::string::npos);
    float_str.erase(float_str.find_last_not_of('.') + 1, std::string::npos);

    return float_str;
}