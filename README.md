# pwmdriver - ROS2 SysFS PWM Driver

A simple PWM driver that uses the Linux Kernel `sysfs` interface to configure PWM devices. The driver also supports PPM outputs (for servos and RC motor controllers).

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
1. Copy the example config file `example.yaml` to your robot configuration directory and edit to suit your application.
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
    outputs:
      motor:
        device_name: pwmchip1
        pwm_port: 7
        period: 24000000
        type: ppm
        ppm_range: [1000000, 1500000, 2000000]
```

Output name - string - name of the output, which becomes part of the topic name, i.e. `motor`, `steering_servo`. Letters, numbers and underscores only.
device_name - string - Name given to the PWM device. This is the `pwmchipX` part of the pathname to the SysFS device: `/sys/class/pwm/pwmchipX`.
pwm_port - int - The port number that will be exported on the PWM device.
period - int - The period of the PWM signal in nanoseconds. For RC servos and motor controllers this is 24000000.
type - string - Options are: `ppm` or `pwm`. The 'type' of PWM signal to output. RC servos and motor controllers needs this to be set to `ppm`.
ppm_range - int[] - Format is either [`minimum`, `zero-point`, `maximum`] or [`minimum`, `maximum`]. These are the signal durations in nanoseconds for
    minimum (0 or -1), zero-point (0) and maximum (1.0). If a zero-point is provided then the port will accept normalised input from -1 to 1, otherwise 0 to 1.
    `[1000000, 1500000, 2000000]` this would cause the setting of the following duty cycles: -1 would be a duty cycle of 1 ms, 0 becomes 1.5 ms and 1 becomes 2 ms.
    This would be used for a motor controller that is capable of reverse.

## Other Information
### Kernel drivers

### udev Rules
