# ros2_control Waveshare servo hardware interface

The `ros2_control` implementation for Waveshare ST series servo motors.

Specifically designed for [Waveshare ST3025 servo motors](https://www.waveshare.com/product/st3025-servo.htm) and their [Bus Servo Adapter](https://www.waveshare.com/product/bus-servo-adapter-a.htm), but should work with all of their ST series motors and controllers.

## TODO

- velocity command interface 
- currently the position interface cannot move counter-clockwise past 0
- position motors should expose position, velocity, and acceleration command interfaces for smoother motion using the joint_trajectory_controller
    - these controls are available in the sync write position function but requires some restructuring 
- software tests
- hardware tests with multiple motors
- ping all motors IDs in the on_init function
- add simulation capabilities

# License

Most of the servo code is from the SCServo_Linux package available on their website.
Waveshare does not include a license in the example files.
When asked, they said to use the GPLv3 license. 

Some of the servo code is from [adityakamath on github](https://github.com/adityakamath/SCServo_Linux).