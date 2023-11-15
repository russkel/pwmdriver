#include <fmt/core.h>

#include "pwmdriver/param_pwmport.hpp"

ParamConfiguredPWMPort::ParamConfiguredPWMPort(rclcpp::Node* node, std::string port_name)
{
    pwm_device = node->declare_parameter(fmt::format("{}.device_path", port_name), "pwmchip0");
    channel = node->declare_parameter(fmt::format("{}.channel", port_name), 0);
    int period = node->declare_parameter(fmt::format("{}.period", port_name), 0);
    std::string port_type = node->declare_parameter(fmt::format("{}.type", port_name), "pwm");
    std::vector<int64_t> ppm_range = node->declare_parameter(fmt::format("{}.range", port_name), std::vector<int64_t> {});
    float trim = node->declare_parameter(fmt::format("{}.trim", port_name), 0.0f);

    RCLCPP_DEBUG(node->get_logger(), "Setting up output: %s [%s channel %d]", port_name.c_str(), pwm_device.c_str(), channel);

    port = std::make_shared<PWMPort>(pwm_device, channel, period);

    if (port_type == "ppm")
        configure_ppm_range(ppm_range);

    port->set_trim(trim);
}

void ParamConfiguredPWMPort::configure_ppm_range(std::vector<int64_t> ppm_range)
{
    if (ppm_range.size() == 3)
        port->configure_ppm(ppm_range[0], ppm_range[1], ppm_range[2]);
    else if (ppm_range.size() == 2)
        port->configure_ppm(ppm_range[0], ppm_range[0], ppm_range[1]);
    else
        throw std::runtime_error("Invalid PPM range provided, needs to be 2 or 3 ints");
}
