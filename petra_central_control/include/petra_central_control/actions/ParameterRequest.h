/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Provides the ROS2-Service client "UserDialog"
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <petra_central_control/actions/base/RosService.h>
#include <petra_core/srv/user_dialog.hpp>
#include <petra_core/msg/dialog_data_type.hpp>

using UserDialog = petra_core::srv::UserDialog;
using DialogDataType = petra_core::msg::DialogDataType;

template <typename T>
class ParameterRequest : public RosService<UserDialog>
{
    static_assert(std::is_same<bool, T>::value ||
                      std::is_same<int, T>::value ||
                      std::is_same<float, T>::value ||
                      std::is_same<std::string, T>::value,
                  "ParameterRequest only for bool, int, float and std::string");

public:
    static const char OPTIONS_SEPARATOR = ',';

    static BT::PortsList providedPorts() { return {BT::OutputPort<T>("value"),
                                                   BT::InputPort<std::string>("title"),
                                                   BT::InputPort<std::string>("msg"),
                                                   BT::InputPort<std::string>("key"),
                                                   BT::InputPort<std::string>("min"),
                                                   BT::InputPort<std::string>("max"),
                                                   BT::InputPort<std::string>("default_value"),
                                                   BT::InputPort<std::string>("options")}; }

    ParameterRequest(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string serviceName() override { return "UserDialog"; }

    void onSend(std::shared_ptr<UserDialog::Request> request) override
    {
        options_.clear();

        getInput("title", request->title);
        getInput("msg", request->msg);
        request->importance = petra_core::srv::UserDialog::Request::NORMAL;

        DialogDataType data_type;

        getInput("key", data_type.key);
        getInput("min", data_type.min);
        getInput("max", data_type.max);

        if (std::is_same<T, bool>::value)
        {
            data_type.type = DialogDataType::BOOL;

            data_type.min = "";
            data_type.max = "";
        }
        else if (std::is_same<T, int>::value)
        {
            data_type.type = DialogDataType::INT;

            try_parse_options(data_type, request->msg);
        }
        else if (std::is_same<T, float>::value)
        {
            data_type.type = DialogDataType::FLOAT;
        }
        else if (std::is_same<T, std::string>::value)
        {
            data_type.type = DialogDataType::STRING;

            try_parse_options(data_type, request->msg);
        }

        getInput("default_value", data_type.default_value);

        request->data_keys.push_back(data_type);
    }

    void onResult(std::shared_future<std::shared_ptr<UserDialog::Response>> response, std::shared_ptr<UserDialog::Request>) override
    {
        auto data_values = response.get()->data_values;

        if (data_values.size() == 1)
        {
            if (std::is_same<T, bool>::value)
            {
                setOutput<bool>("value", data_values.at(0) == "true");
            }
            else if (std::is_same<T, int>::value)
            {
                setOutput<int>("value", stoi(data_values.at(0)));
            }
            else if (std::is_same<T, float>::value)
            {
                setOutput<float>("value", stof(data_values.at(0)));
            }
            else if (std::is_same<T, std::string>::value)
            {
                if (options_.size() > 0)
                {
                    setOutput<std::string>("value", options_[stoi(data_values.at(0)) - 1]);
                }
                else
                {
                    setOutput<std::string>("value", data_values.at(0));
                }
            }
        }
    }

private:
    std::vector<std::string> options_;

    void try_parse_options(DialogDataType &data_type, std::string &msg)
    {
        BT::Optional<std::string> options_input = getInput<std::string>("options");

        if (options_input.has_value())
        {
            auto parts = BT::splitString(options_input.value(), OPTIONS_SEPARATOR);

            std::string option;

            if (msg != "")
            {
                msg += "\n";
            }

            for (size_t index = 0; index < parts.size(); index++)
            {
                option = BT::convertFromString<std::string>(parts[index]);

                options_.push_back(option);

                msg += "[" + std::to_string(index + 1) + "] " + option + "\n";
            }

            msg.pop_back();

            if (options_.size() > 0)
            {
                data_type.type = DialogDataType::INT;
                data_type.min = "1";
                data_type.max = std::to_string(parts.size());
            }
        }
    }
};