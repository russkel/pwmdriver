# pwmdriver - ROS2 SysFS PWM Driver

A simple PWM driver that uses the Linux Kernel `sysfs` interface to configure PWM devices. The driver also supports PPM style outputs (for servos and RC motor controllers).

## Features

* Supports setting PWM duty or PPM compliant duty cycles.
* Set PWM devices using a normalised float value (0 to 1).
* PPM zero-points can be set to support reverse on RC motor controllers, allowing the duty to be set with a normalised float from -1.0 to 1.0.

## Why

When researching how I was going to interface with a `PCA9685` board for a *DIY RoboCar* project, I found that a driver for that hardware exists in the Linux kernel.
This meant I was able to simply use the [SysFS](https://man7.org/linux/man-pages/man5/sysfs.5.html) interface to configure that particular board, and any other PWM devices that kernel driver
support exists for (of which there is a [a lot](https://github.com/torvalds/linux/tree/master/drivers/pwm)). This also includes Single Board Computers such as Raspberry Pi and Rock Pi, where
you can use the PWM port on the GPIO header.

A survey of the available ROS packages didn't show many ROS 2 PWM packages, and none of those used the SysFS interface. So here is my package to solve this.

## Usage

1. Enable the kernel driver for the PWM hardware by configuring the Device Tree.
1. Set up the `udev` rule so the PWM device's SysFS interface can be written to from non-root users.
1. Install/compile this package into your ROS2 install or workspace.
1. Add the `pwmdriver` node to your launch scripts, or run the `pwmdriver` node, making sure to specify your configuration file.
1. Publish the normalised float to the `/pwm/<output name>` topic, setting the duty cycle.
1. Zoom around!

### Messages

`/pwm/<name>`: `std_msgs/msg/Float32` - Duty is set using a `Float32` message. That's all there is to it.

## Configuration

An example output `motor` is shown below:

```yml
pwm_driver:
  ros__params:
    outputs: ["esc", "servo"]
    esc:
      device_path: /sys/class/pwm/pwmchip1
      channel: 7
      period: 2040000
      type: ppm
      range: [1000000, 1500000, 2000000]
    servo:
      device_path: /sys/class/pwm/pwmchip2
      channel: 7
      period: 20000000
      type: ppm
      range: [1000000, 1500000, 2000000]
```


| Parameter     | Type     | Description                                                                                                                              |
|---------------|----------|------------------------------------------------------------------------------------------------------------------------------------------|
| `device_path` | `string` | Path to the PWM device. e.g. `/sys/class/pwm/pwmchip1`.                                                                                  |
| `channel`     | `int`    | The PWM channel that will be used for this output.                                                                                       |
| `period`      | `int`    | The period of the PWM signal in nanoseconds. For RC servos this is `20000000` (20 ms) and motor controllers this is `2040000` (2.04 ms). |
| `type`        | `string` | Options are: `ppm` or `pwm`. The 'type' of PWM signal to output. RC servos and motor controllers needs this to be set to `ppm`.          |
| `range`       | `int[]`  | Format is either [`minimum`, `zero-point`, `maximum`] or [`minimum`, `maximum`].                                                         |
| `trim`        | `float`  | Trim value for the scaled duty output. This value is added to the normalised duty value.                                                 |

### PPM Range

These are the signal durations in nanoseconds for minimum (0 or -1), zero-point (0) and maximum (1.0). If a zero-point is provided then the duty cycle input will accept normalised input from -1.0 to 1.0, otherwise 0.0 to 1.0.  `[1000000, 1500000, 2000000]` this would cause the setting of the following duty cycles: -1 would be a duty cycle of 1 ms, 0 becomes 1.5 ms and 1 becomes 2 ms. This would be used for a motor controller that is capable of reverse.

## Other Information

### udev Rules

By default the SysFS interface for PWM devices is only writable by root. To allow non-root users to write to the interface, a `udev` rule must be used to change the permissions on PWM related events. Check to see the group and permissions on the PWM device. If your OS has a group that is allowed to write to the files then ensure your user is in that group. If not use the provided `udev` rule in the `misc` directory to change the permissions to allow users from the `plugdev` group to write to the PWM device.

### TODO

* `ros2_control` plugin
