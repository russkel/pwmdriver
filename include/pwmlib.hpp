#include <string>
#include <filesystem>
#include <fstream>
#include <stdexcept>


class PWMPort {
public:
    void set_period(int32_t period);
    void set_enabled(bool enable);
    void set_duty_direct(int32_t duty);
    void set_duty_scaled(float duty);
    void deactivate();
    void configure_ppm(int32_t min_point, int32_t zero_point, int32_t max_point);
    void configure_pwm();

    PWMPort(std::string_view pwm_device, int16_t pin_number, int32_t period);

private:
    std::filesystem::path dev_path;
    std::filesystem::path duty_cycle_path;
    std::filesystem::path pwm_pin_path;
    std::ofstream duty_fs;
    int16_t pin_number;
    int32_t pin_period;
    bool pin_ppm = false;
    int32_t pin_ppm_min = 0;
    int32_t pin_ppm_max = 0;
    int32_t pin_ppm_zero = 0;
};