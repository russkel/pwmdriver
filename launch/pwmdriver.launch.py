import launch
from launch_ros.actions import Node


def generate_launch_description():
    pwmdriver = Node(
        package="pwmdriver",
        executable="pwm_driver",
        parameters=[{
            "outputs": ["drive_servo"],
            "drive_servo": {
                "device_path": "/sys/class/pwm/pwmchip1",
                "channel": 0,
                "period": 24000000,
                "type": "ppm",
                "range": [1000000, 1500000, 2000000]
            }
        }],
    )

    return launch.LaunchDescription([
        pwmdriver
    ])
