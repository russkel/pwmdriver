#include <memory>
#include <string>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>
#include <cmath>

#include <fmt/core.h>

#include "pwmdriver/pwmlib.hpp"

using namespace std::chrono_literals;

void PWMPort::set_period(int32_t period) {
    std::ofstream ofs(channel_path / "period");
    if (!ofs.is_open())
        throw std::runtime_error(fmt::format("Cannot open {}. Likely insufficient permissions", (channel_path / "period").string()));
    ofs << period;
    ofs.close();

    channel_period = period;
}

void PWMPort::set_enabled(bool enable) {
    std::ofstream ofs(channel_path / "enable");
    if (!ofs.is_open())
        throw std::runtime_error(fmt::format("Cannot open {}. Likely insufficient permissions", (channel_path / "enable").string()));
    ofs << enable;
    ofs.close();
}

void PWMPort::set_polarity() {
    std::ofstream ofs(channel_path / "polarity");
    if (!ofs.is_open())
        throw std::runtime_error(fmt::format("Cannot open {}. Likely insufficient permissions", (channel_path / "polarity").string()));
    ofs << "normal";
    ofs.close();
}

void PWMPort::set_duty_direct(int32_t duty) {
    duty_fs = std::ofstream(duty_cycle_path);
    if (!duty_fs.is_open())
        throw std::runtime_error("duty_cycle fstream not open, cannot set duty cycle");
    duty_fs << duty;
    duty_fs.close();
}

void PWMPort::set_duty_scaled(float duty) {
    if (-1.0f > duty || duty > 1.0f)
        throw std::runtime_error("Invalid duty provided. Duty must be between -1.0 and 1.0");

    int32_t duty_value;
    if (channel_ppm) {
        if (channel_ppm_zero != channel_ppm_min){
            // if there is a zero point defined, then negative duties are supported
            if (duty < 0.0f)
                duty_value = channel_ppm_min + static_cast<int32_t> (duty * (channel_ppm_zero - channel_ppm_min));
            else
                duty_value = channel_ppm_zero + static_cast<int32_t> (duty * (channel_ppm_max - channel_ppm_zero));

        } else {
            if (std::signbit(duty))
                throw std::runtime_error("Invalid duty provided. Cannot set a negative duty if zero point is also the minimum");
            duty_value = channel_ppm_min + static_cast<int32_t> (duty * (channel_ppm_max - channel_ppm_min));
        }
    } else
        duty_value = static_cast<int32_t> (duty * channel_period);

    set_duty_direct(duty_value);
}

void PWMPort::deactivate() {
    set_enabled(false);

    std::ofstream ofs(dev_path / "unexport");
    if (!ofs.is_open())
        throw std::runtime_error(fmt::format("Cannot open {}. Likely insufficient permissions", (dev_path / "unexport").string()));
    ofs << channel;
    ofs.close();
}

void PWMPort::configure_ppm(int32_t min_point, int32_t zero_point, int32_t max_point) {
    channel_ppm = true;
    channel_ppm_min = min_point;
    channel_ppm_zero = zero_point;
    channel_ppm_max = max_point;

    if (channel_ppm_zero < channel_ppm_min)
        throw std::runtime_error("Output PPM zero point cannot be smaller than min point");

    if (!(channel_ppm_max > channel_ppm_min && channel_ppm_max > channel_ppm_zero))
        throw std::runtime_error("Output PPM max point has to be greater than zero and min points");
}

void PWMPort::configure_pwm() {
    channel_ppm = false;
}

PWMPort::PWMPort(std::string_view pwm_device, int16_t channel_num, int32_t period) {
    dev_path = std::filesystem::path(pwm_device);
    if (!std::filesystem::exists(dev_path))
        throw std::runtime_error(fmt::format("Cannot find PWM device {}", dev_path.string()));

    channel = channel_num;
    channel_path = dev_path / fmt::format("pwm{}", channel);

    if (std::filesystem::exists(channel_path))
        throw std::runtime_error(fmt::format("Channel {} already exported. Could be in use.", channel_path.string()));

    std::ofstream ofs(dev_path / "export");
    if (!ofs.is_open())
        throw std::runtime_error(fmt::format("Cannot open {}. Likely insufficient permissions", (dev_path / "export").string()));
    ofs << channel;
    ofs.close();

    // wait for udev to apply permissions to the newly exported pin
    std::this_thread::sleep_for(500ms);

    if (!std::filesystem::exists(channel_path))
        throw std::runtime_error(fmt::format("Cannot find pwm channel at {}. Export of channel failed", channel_path.string()));

    duty_cycle_path = channel_path / "duty_cycle";

    set_enabled(false);

    if (std::filesystem::exists(channel_path / "polarity"))
        set_polarity();

    set_period(period);
    set_duty_direct(0);
}

PWMPort::~PWMPort() {
    deactivate();
}
