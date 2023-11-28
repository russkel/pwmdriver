#include <functional>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <filesystem>

#include <fmt/core.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "pwmdriver/param_pwmport.hpp"

class PWMDriver : public rclcpp::Node {
public:
    PWMDriver() : Node("pwm_driver")
    {
        RCLCPP_INFO(this->get_logger(), "PWM SysFS Driver starting...");

        auto outputs = this->declare_parameter("outputs", std::vector<std::string> {});

        for (auto port_name : outputs) {
            try {
                auto port = std::make_shared<ParamConfiguredPWMPort>(this, port_name);
                std::string topic = fmt::format("pwm/{}", port_name);
                RCLCPP_INFO(this->get_logger(), "Configured output: %s [%s channel %d] on %s", port_name.c_str(), port->pwm_device.c_str(), port->channel, topic.c_str());

                pwm_outputs[port_name] = port;
                pwm_outputs_subs[port_name] = this->create_subscription<std_msgs::msg::Float32>(topic, rclcpp::QoS(rclcpp::KeepLast(1)),
                    [this, port_name](const std_msgs::msg::Float32::SharedPtr msg) {
                        handle_set_duty(msg, port_name);});

            } catch(std::runtime_error& e) {
                RCLCPP_ERROR(this->get_logger(), "Error occured while setting up output (%s): %s", port_name.c_str(), e.what());
                rclcpp::shutdown();
                return;
            }
        }

        RCLCPP_INFO(this->get_logger(), "PWM outputs are going live");
        activate_outputs();
    }

private:

void handle_set_duty(std_msgs::msg::Float32::ConstSharedPtr msg, const std::string output_name) {
    RCLCPP_DEBUG(this->get_logger(), "Setting duty: %s --> %f", output_name.c_str(), msg->data);
    try {
        pwm_outputs[output_name]->port->set_duty_scaled(msg->data);
    } catch(std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Error occured while setting duty on output (%s): %s", output_name.c_str(), e.what());
    }
}

void activate_outputs() {
    for (auto& output: pwm_outputs) {
        RCLCPP_DEBUG(this->get_logger(), "Activating output %s", output.first.c_str());
        try {
            output.second->port->check();
            output.second->port->set_duty_scaled(0.0);
            output.second->port->set_enabled(true);
        } catch(std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error occured while activating output (%s): %s", output.first.c_str(), e.what());
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "Activated outputs");
}

void deactivate_outputs() {
    for (auto& output: pwm_outputs) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivating output %s", output.first.c_str());
        try {
            output.second->port->deactivate();
        } catch(std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error occured while deactivating output (%s): %s", output.first.c_str(), e.what());
        }
    }
}

std::map<std::string, std::shared_ptr<ParamConfiguredPWMPort>> pwm_outputs;
std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> pwm_outputs_subs;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PWMDriver>());
    rclcpp::shutdown();
    return 0;
}