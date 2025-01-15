# ros2_control Waveshare servo hardware interface

The `ros2_control` implementation for Waveshare ST series servo motors.

Specifically designed for [Waveshare ST3025 servo motors](https://www.waveshare.com/product/st3025-servo.htm) and their [Bus Servo Adapter](https://www.waveshare.com/product/bus-servo-adapter-a.htm), but should work with all of their ST series motors and controllers.

## Set Up

This hardware interface is developed for ros2 Humble.

Testing was done with the bus servo adapter plugged into a Jetson Orin Nano via USB.

The Jetson runs ros2 Humble and Isaac ros insidie a Docker container.

It should work with any system using [ros2_control](https://github.com/ros-controls/ros2_control).

1. Ensure the container can control the USB port

    ```bash
    sudo chmod 666 /dev/ttyACM0
    ```

    ```bash
    sudo usermod -a -G dialout admin
    ```

2. Clone the package into your `src` directory:

    ```bash
    git clone https://github.com/htchr/waveshare_servos.git
    ```

3. Build your workspace.

4. Source your workspace.

## Usage

Reference the `example.launch.py`, `example_controllers.yaml`, and `example.ros2_control.xacro` files to reference how to use this hardware interface in another robot system.

To verify your installation works, launch the example launch file:

```bash
ros2 launch waveshare_servos example.launch.py
```

Move a position-controlled servo with:

```bash
ros2 topic pub --once /joint_trajectory_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, joint_names: ['<joint_name>'], points: [{positions: [<position>], time_from_start: {sec: 1, nanosec: 0}}]}"
```

Move a velocity-controlled servo with:

```bash
ros2 topic pub --once /joint_trajectory_velocity_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, joint_names: ['<joint_name>'], points: [{velocities: [<velocities>], time_from_start: {sec: 1, nanosec: 0}}]}"
```

## Additional Tools

Also included are some helper functions wrapped in ros2 nodes for ease-of-use.

### Change motor ID

To control multiple motors, they will need different IDs.

To set a new ID, plug in 1 motor at a time (make sure to turn off power in between), and run:

```bash
ros2 run waveshare_servos set_id --ros-args -p start_id:=<old> -p new_id:=<new>
```

## TODO

- set 0-point node
- software tests
- hardware tests with multiple motors

## License

Most of the servo code is from the SCServo_Linux package available on their website.
Waveshare does not include a license in the example files.
When asked, they said to use the GPLv3 license. 

Some of the servo code is from [adityakamath on github](https://github.com/adityakamath/SCServo_Linux).