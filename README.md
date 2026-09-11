# ros2_control Waveshare servo hardware interface

The `ros2_control` implementation for Waveshare ST series servo motors.

Specifically designed for [Waveshare ST3025 servo motors](https://www.waveshare.com/product/st3025-servo.htm) and their [Bus Servo Adapter](https://www.waveshare.com/product/bus-servo-adapter-a.htm), but should work with all of their ST series motors and controllers.


## Set Up

This hardware interface is developed for ros2 Jazzy.  
Previous work for ros2 Humble is saved as a checkpoint on the ["humble" branch](https://github.com/htchr/waveshare_servos/tree/humble).

Testing has been done with the bus servo adapter connected via USB to a Jetson Orin Nano or x86 Ubuntu desktop.  
The Jetson ran ros2 Jazzy and Isaac ros inside a Docker container.  
The desktop ran ros2 Jazzy using the devcontainer in this repo.  

It should work with any system using [ros2_control](https://github.com/ros-controls/ros2_control).

### Included Devcontainer

1. Install the "Remote Development" extension pack for VS Code
2. Open the directory in VS Code
3. Select "Reopen in container"

### Direct Install

1. Clone the package into your `src` directory:
    ```bash
    git clone https://github.com/htchr/waveshare_servos.git
    ```
2. Install the dependencies
    ```bash
    rosdep install --from-paths ./src --ignore-src -r -y
    ```
3. Build your workspace.
4. Source your workspace.


## Usage

If you use USB, make sure your user is in the `dialout` group and the port has the correct permissions

```bash
sudo chmod 666 /dev/ttyACM0
sudo usermod -a -G dialout $USER
```

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

Move the velocity-controlled servos with:

```bash
ros2 topic pub --once /joint_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [<velocities>]}"
```

The message carries no joint names.
`data` holds one velocity (rad/s) per joint, in the order of the `joints` list of `joint_velocity_controller` in `example_controllers.yaml` -- not servo id order or URDF order.
For example, with

```yaml
joint_velocity_controller:
  ros__parameters:
    joints:
      - joint4
      - joint3
```

`{data: [1.0, -0.5]}` turns `joint4` at 1.0 rad/s and `joint3` at -0.5 rad/s.
Check the order on a running system with `ros2 param get /joint_velocity_controller joints`.

Each servo keeps turning at its commanded velocity until a new command arrives; send a zero for every joint (e.g. `{data: [0.0, 0.0]}`) to stop them.
A command whose length does not match the `joints` list stops all the velocity-controlled servos and deactivates the controller; reactivate it with `ros2 control switch_controllers --activate joint_velocity_controller`.

## Additional Tools

Also included are some helper functions wrapped in ros2 nodes for ease-of-use.

### Change Motor ID

To control multiple motors, they will need different IDs.

To set a new ID, plug in 1 motor at a time (make sure to turn off power in between), and run:

```bash
ros2 run waveshare_servos set_id --ros-args -p start_id:=<old> -p new_id:=<new>
```

### Set Midpoint

The following command will set the middle position (tick 2048, pi radians, 180 degrees) of a given motor:

```bash
ros2 run waveshare_servos calibrate_midpoint --ros-args -p id:=<id>
```


## TODO

- software tests


## License

Most of the servo code is from the SCServo_Linux package available on their website.
Waveshare does not include a license in the example files.
When asked, they said to use the GPLv3 license. 

Some of the servo code is from [adityakamath on github](https://github.com/adityakamath/SCServo_Linux).
