#include <string>
#include <vector>

#include "pwmlib.hpp"
#include "rclcpp/rclcpp.hpp"

class ParamConfiguredPWMPort {
public:
    ParamConfiguredPWMPort(rclcpp::Node* node, std::string port_name);

    void configure_ppm_range(std::vector<int64_t> ppm_range);

    std::shared_ptr<PWMPort> port;
    std::string pwm_device;
    uint16_t channel;
};