# waveshare_servos

A [ros2_control](https://github.com/ros-controls/ros2_control) `SystemInterface` for
Waveshare/Feetech ST and STS serial bus servos (the SMS/STS protocol) on ROS 2 Jazzy. Its target
hardware is the [Waveshare ST3025 servo](https://www.waveshare.com/product/st3025-servo.htm) and the
Waveshare [Bus Servo Adapter (A)](https://www.waveshare.com/product/bus-servo-adapter-a.htm).
The package has the plugin, four tools, a four-joint example and a test suite that needs no servos.

- Up to 253 servos on one bus. The bus model gives space for about twelve at 100 Hz
  ([Cost per servo](docs/bus-timing.md#cost-per-servo)). No measurement used more than four.
- Position servos (mode 0) and wheels (mode 1). The joint `type` or command interfaces set the mode.
- Multi-turn wheel positions, so `diff_drive_controller` odometry does not jump once per turn.
- Nine state interfaces. Each joint declares the ones it needs, in any order.
- An exclusive port lock (`TIOCEXCL`, `flock`): only root can open the port while the driver has it.
- One sync read per cycle. On the reference bench, `read()` and `write()` use 1.66 ms of 10 ms.

For Humble, use 0.1.0 ([`humble` branch](https://github.com/htchr/waveshare_servos/tree/humble)).

## Set up

Requirements:

- Ubuntu 24.04 and ROS 2 Jazzy, tested with ros2_control 4.48.0 and ros2_controllers 4.42.1.
- ST3025 servos (tested). Other SMS/STS servos can work, but no test used them. The driver does
  not support the SCS/SCSCL series: it refuses `protocol` `scscl`.
- A servo supply: USB does not supply power to the servos. For USB control, the jumper cap of the
  adapter must be on B ([Waveshare wiki](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A))).
- A baud rate of 9600, 19200, 38400, 57600, 115200, 500000 or 1000000. All timing defaults are
  for 1000000 ([Other baud rates](docs/bus-timing.md#other-baud-rates)).
- Your user in `dialout`: run `sudo usermod -aG dialout "$USER"`. Then log out and log in again
  ([Serial port access](docs/setup.md#serial-port-access)).

The package is source-only (no apt package). Build it in a colcon workspace:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b jazzy https://github.com/htchr/waveshare_servos.git   # the default branch holds 0.1.0
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init        # once per machine; skip it if it says the sources list already exists
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y   # no -r: -r hides errors
colcon build --packages-select waveshare_servos
source install/setup.bash   # in each new terminal
```

## Usage

WARNING: Obey these rules before you launch the example on hardware:

- Use the example only on the [reference bench](docs/setup.md#the-reference-bench). The driver can
  write the mode register (EEPROM) of a servo, and change a position servo to a wheel.
- Keep clear of the joints. Torque comes on at activation, and a joint can move toward an old goal.
- Keep the servo supply switch within reach. Only a clean Ctrl-C stops the wheels, and torque stays
  on after shutdown ([Safety](docs/operation.md#safety)).

1. Disconnect the adapter. A misspelt `use_mock_hardware` starts the real driver. Start the example
   on mock hardware. In a second terminal, show the controllers:
   ```bash
   ros2 launch waveshare_servos example.launch.py use_mock_hardware:=true gui:=false
   ros2 control list_controllers
   ```
2. Connect the adapter and switch on the servo supply. Find the servos:
   `ros2 run waveshare_servos scan --ros-args -p port:=/dev/ttyACM0`
3. Launch the example on the bench:
   `ros2 launch waveshare_servos example.launch.py gui:=false`
4. Move `joint1` to 0.6 rad. `-t 3 -r 10` sends the message three times, because a new
   `ros2 topic pub` process can lose its first message:
   ```bash
   ros2 topic pub -t 3 -r 10 /joint_trajectory_position_controller/joint_trajectory \
     trajectory_msgs/msg/JointTrajectory \
     "{joint_names: [joint1], points: [{positions: [0.6], time_from_start: {sec: 2}}]}"
   ```
5. Turn `joint3` at 1.0 rad/s and `joint4` at -0.5 rad/s. Then stop the two wheels:
   ```bash
   ros2 topic pub -t 3 -r 10 /joint_velocity_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [1.0, -0.5]}"
   ros2 topic pub -t 3 -r 10 /joint_velocity_controller/commands std_msgs/msg/Float64MultiArray \
     "{data: [0.0, 0.0]}"
   ```

[Set up and first run](docs/setup.md) gives the full procedure. For your own robot, see
[Adapt the example](docs/configuration.md#adapt-the-example).

## Tools

Stop the controller manager before you use a tool. While another process holds the port, a tool
sends nothing (exit code 1). Read the safety notes in [Command-line tools](docs/tools.md).

- `scan` lists each servo that answers on ids 0 to 253. It only reads.
- `set_id` changes the id of a servo. New servos all have the same id, so connect only that servo.
- `calibrate_midpoint` makes the present position read tick 2048, and leaves the torque off.
- `factory_reset` sets the EEPROM of one servo to factory values, except the id. The baud rate goes
  to 1000000 and the torque goes off. Save the output: it is the only record of the old values.

```bash
ros2 run waveshare_servos scan --ros-args -p port:=/dev/ttyACM0
ros2 run waveshare_servos set_id --ros-args -p start_id:=<old> -p new_id:=<new>
ros2 run waveshare_servos calibrate_midpoint --ros-args -p id:=<id>
ros2 run waveshare_servos factory_reset --ros-args -p id:=<id>
```

## Documentation

- [Set up and first run](docs/setup.md): the first run, a differential base, real-time scheduling.
- [Configure a robot](docs/configuration.md): parameters, interfaces, limits and status bits.
- [Bus timing](docs/bus-timing.md): cycle cost, `feedback_mode`, `io_timeout_ms`, bus totals.
- [Operation](docs/operation.md): start-up, faults, recovery, troubleshooting and known issues.
- [Command-line tools](docs/tools.md): safe use of the four tools, their parameters and exit codes.
- [Bench check](docs/bench-check.md): the hardware-in-the-loop check on the reference bench.
- [Driver design](docs/design.md): how the driver and the tools work, for maintainers.
- [Development and release](docs/development.md): tests, release checklist and versioning.

## Changes

[CHANGELOG.rst](CHANGELOG.rst) lists the changes of each version. To upgrade, read the sections
"Breaking changes (upgrading from 0.1.0)" and "Behaviour changes with an unchanged description".
Most 0.1.0 users meet these four changes:

- The wheel controller of the example is `joint_velocity_controller` (step 5 of [Usage](#usage)).
- The tools refuse `device_port` and `baud_rate` (exit code 64). Use `port` and `baudrate`.
- `allow_missing_servos` is `false` by default. If a servo does not answer, `on_configure` fails.
- The `effort` state interface (N m) replaces the deprecated `torque` state interface.

## Contributing

Send issues and pull requests to the [repository](https://github.com/htchr/waveshare_servos).
A status bit that you confirm on hardware is especially welcome: tell what you did and what you saw.
[Development rules](docs/development.md#development-rules) gives the rules for a change.

## License

The package license is GPL-3.0-or-later ([LICENSE](LICENSE)). The author asked Waveshare about the
license, and Waveshare said to use GPLv3. The servo packet layer is the Feetech SCServo library,
with two additions from [adityakamath/SCServo_Linux](https://github.com/adityakamath/SCServo_Linux)
and one local fix. `include/visibility_controls.h` and `bringup/launch/example.launch.py` keep their
Apache-2.0 headers ([LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt)).
[THIRD_PARTY.md](THIRD_PARTY.md) gives the origin, the changes, the checksums and the licenses.
