#ifndef WAVESHARE_SERVOS_HPP_
#define WAVESHARE_SERVOS_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>
#include <string>

#include "position_unwrapper.hpp"
#include "units.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_controls.h"  // NOLINT(build/include_subdir)
#include "servo_bus.hpp"

namespace waveshare_servos
{

// Jazzy's hardware_interface_type_values.hpp has no constant for these three names; the
// other six come from that header (HW_IF_POSITION, HW_IF_VELOCITY, ...).
constexpr char HW_IF_VOLTAGE[] = "voltage";
constexpr char HW_IF_LOAD[] = "load";
constexpr char HW_IF_STATUS[] = "status";

class WaveshareServos : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WaveshareServos)

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  WAVESHARE_SERVOS_PUBLIC
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
  override;

  WAVESHARE_SERVOS_PUBLIC
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
  override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  // Parse and validate the <hardware><param> block into the members below. Logs one FATAL and
  // returns ERROR at the first bad value; no bus access (the port is opened in on_configure).
  hardware_interface::CallbackReturn read_hardware_parameters();
  // One unicast feedback round trip that refreshes every state cache of joint i; false if the
  // servo did not answer. Used by the activation seed, the park and the per-servo transport.
  bool feedback(size_t i);
  // This cycle's sample for joint i: from read()'s sync-read burst when that path is active,
  // else one unicast round trip. Both paths end in apply_feedback().
  bool feedback_from_cycle(size_t i);
  // Decode one feedback block into joint i's state caches. Both transports use it, so they
  // publish the same values for the same bytes.
  void apply_feedback(size_t i, const FeedbackBlock & b);
  // Rebuild the read group (r_ids_, r_js_, r_slot_, r_blocks_) from present_, in URDF order.
  // No bus traffic: read() calls it on the real-time path (build_groups() talks to the bus).
  void build_read_group();
  // Once per activation, choose the read() transport: one INST_SYNC_READ burst or one read
  // per servo. Returns false only when feedback_mode is 'sync_read' and the burst goes
  // unanswered. Changes no state cache, unwrapper or measured_.
  bool probe_sync_read();
  // Log the 'bus totals:' INFO line, at most once per activation (on_deactivate, on_error).
  // Tests and test/hil/hil_gates.py parse it: keep the format.
  // See docs/bus-timing.md, "Bus totals line".
  void log_bus_totals();
  // set the servo's operating mode, unlocking the EPROM only if it needs changing
  bool set_mode(u8 id, u8 mode);
  // Write register 41 (acceleration, SRAM) of joint i if it is a present wheel; else do
  // nothing. A servo power cycle clears SRAM, so four call sites write it again.
  // See docs/design.md, "Wheel acceleration".
  void write_wheel_acceleration(size_t i);
  // (re)build the present-only command groups and their record arrays
  void build_groups();
  // look up the framework-owned interface handles of every joint once (on_configure), so read()
  // and write() never hash an interface name
  void cache_handles();
  // copy the controllers' commands of joint i from the command handles into pos_cmds_/vel_cmds_;
  // wait=false on the real-time path, where a busy handle leaves the previous value in place
  void pull_commands(size_t i, bool wait);
  // publish pos_cmds_[i] / vel_cmds_[i] to the position / velocity command handle of joint i, if
  // the joint has one (blocking: lifecycle only)
  void push_position_command(size_t i);
  void push_velocity_command(size_t i);
  // publish the state caches of joint i to the state handles the URDF asked for, and only those
  void push_states(size_t i, bool wait);
  // the value of one state interface of joint i, read straight out of its cache
  double state_value(size_t i, StateKind kind) const;
  // the per-joint constants decode() needs: the joint's own offset and sign, and the three
  // hardware-level scales
  FeedbackScales scales_of(size_t i) const;
  // turn pos_cmds_/vel_cmds_ into goals for the servos in the command groups and send them; write()
  // after it has taken the controllers' commands from the handles
  void send_commands(const rclcpp::Duration & period);
  // Stop the wheels, park the position joints where they are, and send that once.
  // false (on_deactivate): send through the handles and write(). true (on_shutdown,
  // on_error): park only at a measured position, send directly and prune the command
  // groups, so close the port next. See docs/design.md, "Lifecycle".
  void stop_and_park(bool at_measured_positions);
  // stop_and_park(true) if the port is open, then close_port(); a failure to park is logged and
  // never keeps the port open or escapes
  void park_and_close();
  // Whether position is more than one encoder step outside joint i's limits; always false
  // for a velocity (wheel) joint.
  bool outside_limits(size_t i, double position) const;
  // Restart every unwrapped count: a count belongs to one open port.
  void reset_unwrap();
  // the same for one joint, throttle counter included, so the two can never fall out of step
  void reset_unwrap(size_t i);
  // Joint i's multi-turn tick count after this raw sample (raw itself if the joint does not
  // unwrap). Call it only for a sample that arrived; apply_feedback() is the only caller.
  int64_t unwrap_ticks(size_t i, int32_t raw);
  // Throttled WARN when a silence was long enough for joint i to turn half a revolution
  // unseen: the one silent, permanent error of an unwrapped count.
  void note_unwrap_gap(size_t i, int missed, double period_s);
  // the joint's speed CEILING in rad/s, which is what the gap warning has to reason with: the
  // measured speed of a servo that is not answering is unknown by construction
  double max_speed_rad(size_t i) const;
  // close the serial port and release the command buffers; safe to call in any state
  void close_port();
  // One FATAL that says in user terms why the port could not be opened. Every on_configure
  // failure logs, closes the port and returns FAILURE.
  void log_open_failure(const OpenResult & result) const;

  // framework-owned handles of one joint, looked up once by name
  struct JointHandles
  {
    // The joint's declared state interfaces, in description order; read() publishes only
    // these, so it never touches a null handle.
    std::vector<std::pair<StateKind, hardware_interface::StateInterface::SharedPtr>> states;
    // null when the joint does not declare that command interface in the URDF
    hardware_interface::CommandInterface::SharedPtr position_cmd;
    hardware_interface::CommandInterface::SharedPtr velocity_cmd;
  };
  std::vector<JointHandles> handles_;

  // how a joint is driven: a position joint runs the servo's own profile to a goal position
  // (mode 0), a velocity joint is a closed-loop wheel (mode 1)
  enum class JointType : uint8_t { position, velocity };

  // the per-joint configuration read out of the URDF once, in on_init, and read-only afterwards.
  // One entry per <joint>, in the order the description lists them. pos_min / pos_max are the
  // position command limits from the ros2_control min/max params, +/-inf where none is given.
  struct JointConfig
  {
    u8 id = 0;                                                    // 1..253, unique in this block
    JointType type = JointType::position;
    bool unwrap = false;                                          // default: type == velocity
    double offset = 0.0;                                          // rad, SERVO frame
    double sign = 1.0;                                            // -1.0 when inverted="true"
    double pos_min = -std::numeric_limits<double>::infinity();    // rad, JOINT frame
    double pos_max = +std::numeric_limits<double>::infinity();    // rad, JOINT frame
    u16 max_speed_counts = 6000;                                  // GOAL_SPEED magnitude ceiling
    u8 acc_counts = 150;                                          // SMS_STS_ACC register value
  };
  std::vector<JointConfig> joints_;

  // <hardware> parameters, all optional. read_hardware_parameters() sets them from
  // waveshare_servos::defaults (src/driver_defaults.hpp); these initializers mirror it.
  int baudrate_ = 1000000;
  std::string port_ = "/dev/ttyACM0";   // /dev/ttyTHS1 if using UART
  // the only door to the serial bus: the vendored packet layer plus exclusive access to the tty.
  // It knows whether it is open, so there is no separate port_open_ flag to fall out of step.
  ServoBus bus_;
  int encoder_steps_ = 4096;
  // One overall budget per readSCS() call (one transaction), not per select() or per servo:
  // one silent servo costs this much per cycle, plus a 2 ms drain on the sync path.
  // See docs/bus-timing.md, "Transaction timeout".
  uint32_t io_timeout_ms_ = 5;   // assigns into SCSerial::IOTimeOut, which is wider
  int ping_attempts_ = 3;
  int max_read_fails_ = 50;
  // false (default): on_configure fails if a declared servo does not answer Ping.
  bool allow_missing_servos_ = false;
  // reserved for a future SCS/SCSCL path and intentionally unread today: only 'sms_sts' is
  // accepted, and it is stored lower-cased so the configuration line always prints one spelling
  std::string protocol_ = "sms_sts";
  // 'auto', 'sync_read' or 'per_servo', lower-cased. Holds the effective mode: on_init may
  // change 'auto' to 'per_servo' when a user-set io_timeout_ms is too short for a sync read.
  // See docs/bus-timing.md, "Feedback mode".
  std::string feedback_mode_ = "auto";
  // ReadCurrent is unitless; these two turn its counts into amperes and then into a torque
  double current_per_count_a_ = 0.006;
  double torque_constant_nm_per_a_ = 0.8825985;   // 9.0 kgf cm/A, in N m/A
  // The same constant in kgf cm/A, for the deprecated `torque` interface; set in on_init.
  double torque_constant_kgfcm_per_a_ = 9.0;
  // command variables: the last commands pulled from the command handles (and what the
  // hardware itself commands in on_activate/on_deactivate); write() and read() use only these
  std::vector<double> pos_cmds_;
  std::vector<double> vel_cmds_;
  // Latest measurement, published to the state handles. All nine come from one feedback
  // round trip and are always computed; a joint publishes only the ones its URDF declares.
  std::vector<double> pos_states_;      // rad, joint frame
  std::vector<double> vel_states_;      // rad/s, joint frame
  std::vector<double> eff_states_;      // N m, a magnitude (inverted does not flip it)
  std::vector<double> cur_states_;      // A, a magnitude (inverted does not flip it)
  std::vector<double> volt_states_;     // V
  std::vector<double> temp_states_;     // deg C
  std::vector<double> load_states_;     // fraction of full PWM, about -1.023 .. +1.023
  std::vector<double> status_states_;   // bitmask 0..255, integer-valued, or NaN for "no reply"
  std::vector<double> torq_states_;     // kgf cm, the deprecated alias of eff_states_
  // the status byte exactly as feedback() captured it, before anything else could overwrite
  // SCS::Error; the fault-edge block in read() and status_states_ both come from here
  std::vector<u8> status_bytes_;
  // Where a joint found outside its position limits (by on_activate or a park) is held until
  // it is commanded inside them; NaN for every other joint.
  std::vector<double> hold_pos_;
  // which servos answered Ping; absent ones are never put on the bus
  std::vector<bool> present_;
  std::vector<int> read_fails_;
  // Consecutive write() cycles whose sync write was refused (closed port or bad id, so
  // nothing was sent); throttles the ERROR.
  int write_refusals_ = 0;
  std::vector<u8> last_error_;
  // whether pos_states_ holds a position the servo returned since on_configure, rather than a
  // stand-in (NaN, on_activate's 0.0, or read()'s mirror of the command of an absent servo)
  std::vector<bool> measured_;
  // One unwrapper per joint (used or not), and each joint's count of possible lost-revolution
  // gaps, which throttles the WARN.
  std::vector<PositionUnwrapper> unwrap_;
  std::vector<int> unwrap_gap_warns_;
  // command groups containing only servos that are present, with the joint index each maps to
  std::vector<u8> p_ids_;
  std::vector<size_t> p_js_;
  std::vector<u8> v_ids_;
  std::vector<size_t> v_js_;
  // last non-zero control period, used to pace the goal speed
  double last_period_ = 0.01;
  // One goal record per servo in p_ids_/v_ids_, refilled every cycle; the send does not
  // modify them. Wheel records carry no acceleration (register 41 is written separately).
  std::vector<GoalPosition> p_goals_;
  std::vector<GoalSpeed> v_goals_;
  // The read group: present joints in URDF order, apart from the command groups, because a
  // dead id costs a sync read a full timeout per cycle but a sync write only its record bytes.
  // r_ids_[k] is asked for, r_js_[k] is its joint, r_blocks_[k] gets its reply.
  // See docs/design.md, "Read cycle".
  std::vector<u8> r_ids_;
  std::vector<size_t> r_js_;
  std::vector<FeedbackBlock> r_blocks_;
  // Joint index -> slot in r_ids_, or kNoSlot. Sync-read replies arrive in request order
  // (measured), so slot k holds the reply of r_ids_[k]. See docs/design.md, "Sync read".
  std::vector<size_t> r_slot_;
  static constexpr size_t kNoSlot = std::numeric_limits<size_t>::max();
  // Set when read() drops a servo. The group is rebuilt after the loop: a rebuild during it
  // would point later joints at another servo's sample.
  bool read_group_dirty_ = false;
  // Whether read() uses the sync-read burst. Only the activation probe sets it, so it never
  // changes mid-activation; the drop policy handles a failing servo.
  bool sync_read_active_ = false;
  // Per-joint polls and failures of read() since on_activate (lifecycle reads not counted).
  // read_failures_ is the per-servo tail of the 'bus totals:' line; read_attempts_ is not
  // logged. See docs/bus-timing.md, "Bus totals line".
  std::vector<uint64_t> read_attempts_;
  std::vector<uint64_t> read_failures_;
  // read() calls since activation; log_bus_totals() logs nothing while it is 0.
  uint64_t read_cycles_ = 0;
  // The aggregates of the 'bus totals:' line: one transaction per wrapper read call (one
  // sync_read_feedback() per cycle, or one read_feedback_one() per polled joint) and at most
  // one failure per transaction. See docs/bus-timing.md, "Bus totals line".
  uint64_t read_transactions_ = 0;
  uint64_t read_transaction_failures_ = 0;
  // Longest failure run and number of dropped servos since activation: a rate alone cannot
  // tell lost replies from a servo that is going away.
  uint32_t worst_consecutive_ = 0;
  uint32_t dropped_count_ = 0;
  bool read_stats_reported_ = false;
};

}  // namespace waveshare_servos

#endif  // WAVESHARE_SERVOS_HPP_
