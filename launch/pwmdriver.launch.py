import launch
from launch_ros.actions import Node


def generate_launch_description():
    pwmdriver = Node(
        package="pwmdriver",
        executable="pwm_driver",
        parameters=[{
            "outputs": ["esc", "servo"],
            "esc": {
                "device_path": "/sys/class/pwm/pwmchip1",
                "channel": 0,
                "period": 2040000,
                "type": "ppm",
                "range": [1000000, 1500000, 2000000]
            },
            "servo": {
                "device_path": "/sys/class/pwm/pwmchip2",
                "channel": 0,
                "period": 20000000,
                "type": "ppm",
                "range": [1000000, 1500000, 2000000]
            }
        }],
    )

    return launch.LaunchDescription([
        pwmdriver
    ])
