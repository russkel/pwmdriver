#include <memory>
#include <string>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>
#include <cmath>

#include <fmt/core.h>

#include "pwmlib.hpp"

using namespace std::chrono_literals;

void PWMPort::set_period(int32_t period) {
    std::ofstream ofs(pwm_pin_path / "period");
    if (!ofs.is_open())
        throw std::runtime_error(fmt::format("Cannot open {}. Likely insufficient permissions", (pwm_pin_path / "period").string()));
    ofs << period;
    ofs.close();

    pin_period = period;
}

void PWMPort::set_enabled(bool enable) {
    std::ofstream ofs(pwm_pin_path / "enable");
    if (!ofs.is_open())
        throw std::runtime_error(fmt::format("Cannot open {}. Likely insufficient permissions", (pwm_pin_path / "enable").string()));
    ofs << enable;
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
    if (pin_ppm) {
        if (pin_ppm_zero != pin_ppm_min){
            // if there is a zero point defined, then negative duties are supported
            if (duty < 0.0f)
                duty_value = pin_ppm_min + static_cast<int32_t> (duty * (pin_ppm_zero - pin_ppm_min));
            else
                duty_value = pin_ppm_zero + static_cast<int32_t> (duty * (pin_ppm_max - pin_ppm_zero));

        } else {
            if (std::signbit(duty))
                throw std::runtime_error("Invalid duty provided. Cannot set a negative duty if zero point is also the minimum");
            duty_value = pin_ppm_min + static_cast<int32_t> (duty * (pin_ppm_max - pin_ppm_min));
        }
    } else
        duty_value = static_cast<int32_t> (duty * pin_period);

    set_duty_direct(duty_value);
}

void PWMPort::deactivate() {
    set_enabled(false);

    std::ofstream ofs(dev_path / "unexport");
    if (!ofs.is_open())
        throw std::runtime_error(fmt::format("Cannot open {}. Likely insufficient permissions", (dev_path / "unexport").string()));
    ofs << pin_number;
    ofs.close();
}

void PWMPort::configure_ppm(int32_t min_point, int32_t zero_point, int32_t max_point) {
    pin_ppm = true;
    pin_ppm_min = min_point;
    pin_ppm_zero = zero_point;
    pin_ppm_max = max_point;

    if (pin_ppm_zero < pin_ppm_min)
        throw std::runtime_error("Output PPM zero point cannot be smaller than min point");

    if (!(pin_ppm_max > pin_ppm_min && pin_ppm_max > pin_ppm_zero))
        throw std::runtime_error("Output PPM max point has to be greater than zero and min points");
}

void PWMPort::configure_pwm() {
    pin_ppm = false;
}

PWMPort::PWMPort(std::string_view pwm_device, int16_t pin_num, int32_t period) {
    dev_path = std::filesystem::path("/sys/class/pwm") / pwm_device;
    if (!std::filesystem::exists(dev_path))
        throw std::runtime_error(fmt::format("Cannot find PWM device {}", dev_path.string()));

    pin_number = pin_num;
    std::ofstream ofs(dev_path / "export");
    if (!ofs.is_open())
        throw std::runtime_error(fmt::format("Cannot open {}. Likely insufficient permissions", (dev_path / "export").string()));
    ofs << pin_number;
    ofs.close();

    // wait a few ms for udev to apply permissions to the newly exported pin
    std::this_thread::sleep_for(500ms);

    pwm_pin_path = dev_path / fmt::format("pwm{}", pin_number);

    if (!std::filesystem::exists(pwm_pin_path))
        throw std::runtime_error(fmt::format("Cannot find pwm output {}. Export of pin failed", pwm_pin_path.string()));

    duty_cycle_path = pwm_pin_path / "duty_cycle";

    set_enabled(false);
    set_period(period);
    set_duty_direct(0);
}

