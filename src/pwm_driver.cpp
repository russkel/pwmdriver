#include <functional>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <filesystem>

#include <fmt/core.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "pwmlib.hpp"

class PWMDriver : public rclcpp::Node {
public:
    PWMDriver() : Node("pwm_driver")
    {
        RCLCPP_INFO(this->get_logger(), "PWM SysFS Driver starting...");

        auto outputs = this->declare_parameter("outputs", std::vector<std::string> {});

        for (auto port_name : outputs) {
            // TODO: check if the port is already used on that particular device

            std::string pwm_device = this->declare_parameter(fmt::format("{}.device_path", port_name), "pwmchip0");
            unsigned int channel = this->declare_parameter(fmt::format("{}.channel", port_name), 0);
            int period = this->declare_parameter(fmt::format("{}.period", port_name), 0);
            std::string port_type = this->declare_parameter(fmt::format("{}.type", port_name), "pwm");
            std::vector<int64_t> ppm_range = this->declare_parameter(fmt::format("{}.range", port_name), std::vector<int64_t> {});

            std::string topic = fmt::format("pwm/{}", port_name);

            RCLCPP_INFO(this->get_logger(), "Setting up output: %s [%s channel %d] on %s", port_name.c_str(), pwm_device.c_str(), channel, topic.c_str());

            try {
                std::shared_ptr<PWMPort> port = std::make_shared<PWMPort>(pwm_device, channel, period);
                pwm_outputs[port_name] = port;

                if (port_type == "ppm")
                    configure_ppm_range(port, ppm_range);

                pwm_outputs_subs[port_name] = this->create_subscription<std_msgs::msg::Float32>(topic, rclcpp::SystemDefaultsQoS(),
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

void configure_ppm_range(std::shared_ptr<PWMPort> port, std::vector<int64_t> ppm_range){
    if (ppm_range.size() == 3)
        port->configure_ppm(ppm_range[0], ppm_range[1], ppm_range[2]);
    else if (ppm_range.size() == 2)
        port->configure_ppm(ppm_range[0], ppm_range[0], ppm_range[1]);
    else
        throw std::runtime_error("Invalid PPM range provided, needs to be 2 or 3 ints");
}

void handle_set_duty(std_msgs::msg::Float32::ConstSharedPtr msg, const std::string output_name) {
    RCLCPP_DEBUG(this->get_logger(), "Setting duty: %s --> %f", output_name.c_str(), msg->data);
    try {
        pwm_outputs[output_name]->set_duty_scaled(msg->data);
    } catch(std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Error occured while setting duty on output (%s): %s", output_name.c_str(), e.what());
    }
}

void activate_outputs() {
    for (auto& output: pwm_outputs) {
        RCLCPP_DEBUG(this->get_logger(), "Activating output %s", output.first.c_str());
        try {
            output.second->set_duty_scaled(0.0);
            output.second->set_enabled(true);
        } catch(std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error occured while activating output (%s): %s", output.first.c_str(), e.what());
        }
    }
}

void deactivate_outputs() {
    for (auto& output: pwm_outputs) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deactivating output %s", output.first.c_str());
        try {
            output.second->deactivate();
        } catch(std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error occured while deactivating output (%s): %s", output.first.c_str(), e.what());
        }
    }
}

std::map<std::string, std::shared_ptr<PWMPort>> pwm_outputs;
std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> pwm_outputs_subs;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PWMDriver>());
    rclcpp::shutdown();
    return 0;
}