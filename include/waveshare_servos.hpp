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

// Jazzy names no constant for these three
// (hardware_interface/types/hardware_interface_type_values.hpp), so the driver declares its own and
// each string is spelled exactly once. The other six come from that header: HW_IF_POSITION,
// HW_IF_VELOCITY, HW_IF_EFFORT, HW_IF_CURRENT, HW_IF_TEMPERATURE and HW_IF_TORQUE.
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
  // one round trip that refreshes every state interface of joint i from the servo's
  // feedback block; returns false if the servo did not answer. Unicast, always: it is the
  // activation seeding read, the park read and the per-servo fallback, never a one-id burst
  // (PHASE3 2.69, F22)
  bool feedback(size_t i);
  // This cycle's sample for joint i -- out of the burst read() already made when the sync path is
  // live, otherwise one unicast round trip right here (PHASE3 3.27). The only selector: the two
  // transports differ in nothing else, because both end in apply_feedback().
  bool feedback_from_cycle(size_t i);
  // Everything that happens to joint i once its sample has arrived, whichever transport brought
  // it. One function, so the two paths publish the same nine doubles for the same 15 bytes by
  // construction rather than by inspection (PHASE3 2.60, 2.63, F9).
  void apply_feedback(size_t i, const FeedbackBlock & b);
  // Rebuild r_ids_ / r_js_ / r_slot_ and size r_blocks_ from present_ and joints_, in URDF order.
  // Bus-free on purpose, and deliberately NOT build_groups(): that one calls set_mode() for every
  // id, which is a readByte of register 33 and possibly an EPROM unlock/write/lock, and read()
  // calls this from the real-time path on the drop edge (PHASE3 2.64).
  void build_read_group();
  // Decide, once per activation, whether read() uses one INST_SYNC_READ or one FeedBack per
  // servo, and say so in the log. Returns false when feedback_mode is 'sync_read' and the bus did
  // not answer, which is the one case that must fail activation instead of degrading quietly
  // (PHASE3 2.74, R6). No side effects on a state cache, an unwrapper or measured_.
  bool probe_sync_read();
  // The one INFO of PHASE3 L5, emitted from on_deactivate after the park and from on_error. Its
  // grammar is parsed by test/hil/hil_gates.py, so it is frozen (PHASE3 5.14, R16).
  void log_bus_totals();
  // set the servo's operating mode, unlocking the EPROM only if it needs changing
  bool set_mode(u8 id, u8 mode);
  // Write joint i's acceleration register (41) if it is a wheel that is on the bus; a no-op for
  // every other joint. Register 41 is SRAM (include/SMS_STS.h:39-48), so it needs no EPROM unlock
  // and survives torque cycles, sync writes and idling -- but not a servo power cycle [P2 Q2],
  // which is why this is called from all FOUR places a servo can have restarted (PHASE3 1.24) and
  // not only from on_configure. A position joint needs none of it: its acceleration is byte 0 of
  // every 7-byte goal record, so it repairs itself on the next cycle (PHASE3 1.21).
  // One unicast writeByte + Ack, measured at 0.594 ms mean / 0.724 ms p99 [P2 Q1]; two of the four
  // sites are off the real-time path and the other two are edges, never a per-cycle cost.
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
  // zero the wheel commands, park the position joints where they are and send that once.
  // Without at_measured_positions (on_deactivate) every command reaches its handle where it is set
  // and write() takes the commands from the handles, as when the controllers shared this memory.
  // at_measured_positions is for on_shutdown and on_error, which can run on a component that was
  // not activated since it was configured: each position joint is parked only at a measured
  // position, held there even outside its limits, a servo that has not returned a position since
  // on_configure or that read() dropped gets no goal at all, and the goals are sent from these
  // values rather than taken back from the handles. That drops such servos from the command groups,
  // so close the port next.
  void stop_and_park(bool at_measured_positions);
  // stop_and_park(true) if the port is open, then close_port(); a failure to park is logged and
  // never keeps the port open or escapes
  void park_and_close();
  // whether a position of joint i lies more than one encoder step outside its position limits;
  // always false for a continuous joint, which has no limits to be outside of (PHASE2_SPEC 8.7)
  bool outside_limits(size_t i, double position) const;
  // start every unwrapped count again: the count belongs to one measurement session on one open
  // port, and to one activation within it (PHASE2_SPEC 8.4)
  void reset_unwrap();
  // the same for one joint, throttle counter included, so the two can never fall out of step
  void reset_unwrap(size_t i);
  // the continuous, multi-turn tick count of joint i after this raw sample; the identity when the
  // joint does not unwrap. Called only from feedback(), and only for a sample that really arrived
  // (PHASE2_SPEC 8.9)
  int64_t unwrap_ticks(size_t i, int32_t raw);
  // one WARN, throttled, when this cycle followed a silence long enough that the joint could have
  // turned more than half a revolution unseen -- the one way an unwrapped count can go wrong
  // silently and permanently (PHASE2_SPEC 8.5)
  void note_unwrap_gap(size_t i, int missed, double period_s);
  // the joint's speed CEILING in rad/s, which is what the gap warning has to reason with: the
  // measured speed of a servo that is not answering is unknown by construction
  double max_speed_rad(size_t i) const;
  // close the serial port and release the command buffers; safe to call in any state
  void close_port();
  // one FATAL naming why the bus could not be taken, in the user's terms rather than the errno's;
  // every on_configure failure path logs, closes the port and returns FAILURE (PHASE2_SPEC 6.3)
  void log_open_failure(const OpenResult & result) const;

  // framework-owned handles of one joint, looked up once by name
  struct JointHandles
  {
    // the joint's state interfaces in the order the description lists them; only the ones it
    // declares are here, so read() publishes exactly what the URDF asked for and never touches a
    // null handle
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

  // The hardware parameters of the <hardware> block, all optional. read_hardware_parameters()
  // seeds every one of them from waveshare_servos::defaults (src/param_parsing.hpp), the single
  // source of truth these initializers mirror, and overwrites only what the description declares.
  int baudrate_ = 1000000;
  std::string port_ = "/dev/ttyACM0";   // /dev/ttyTHS1 if using UART
  // the only door to the serial bus: the vendored packet layer plus exclusive access to the tty.
  // It knows whether it is open, so there is no separate port_open_ flag to fall out of step.
  ServoBus bus_;
  int encoder_steps_ = 4096;
  // The timeout is one overall budget per readSCS() call, not one per select() and not one per
  // servo: src/SCSerial.cpp:141,146-147 set a single timeval, and the while(1) at :150-153 passes
  // that same struct to every select(), which decrements it in place and never restarts it. So a
  // transaction that never completes costs exactly this once -- measured at 1.00-1.03x the setting
  // across 2..50 ms [P1 Q7]. That makes it the price of one non-answering servo per cycle, not a
  // per-servo safety margin: at 5 ms a dead servo costs 5.2 ms of a 10 ms period [P3 Q3], at the
  // library's stock 100 ms it cost ten whole periods, which is what held the Phase 1 loop down to
  // ~1.1 Hz (PHASE3 3.5).
  uint32_t io_timeout_ms_ = 5;   // assigns into SCSerial::IOTimeOut, which is wider
  int ping_attempts_ = 3;
  int max_read_fails_ = 50;
  // whether a servo that does not answer Ping leaves the component configurable (PHASE2_SPEC 6);
  // false is the deliberate Phase 2 behaviour change
  bool allow_missing_servos_ = false;
  // reserved for a future SCS/SCSCL path and intentionally unread today: only 'sms_sts' is
  // accepted, and it is stored lower-cased so the configuration line always prints one spelling
  std::string protocol_ = "sms_sts";
  // Which transport read() asks for: 'auto' (sync read if the activation probe answers),
  // 'sync_read' (never demote, so a firmware that ignores INST_SYNC_READ fails loudly instead of
  // running slowly in silence) or 'per_servo' (PHASE3 2.71-2.72). Stored lower-cased like
  // protocol_. read_hardware_parameters() may latch it to 'per_servo' when a user-set
  // io_timeout_ms cannot carry a burst of this many servos (2.82), so it holds the mode the driver
  // will actually use, not the word the description carried.
  std::string feedback_mode_ = "auto";
  // ReadCurrent is unitless; these two turn its counts into amperes and then into a torque
  double current_per_count_a_ = 0.006;
  double torque_constant_nm_per_a_ = 0.8825985;   // 9.0 kgf cm/A, in N m/A
  // the same constant in the unit the deprecated `torque` interface keeps (D2); computed once in
  // on_init from torque_constant_nm_per_a_, never parsed
  double torque_constant_kgfcm_per_a_ = 9.0;
  // command variables: the last commands pulled from the command handles (and what the
  // hardware itself commands in on_activate/on_deactivate); write() and read() use only these
  std::vector<double> pos_cmds_;
  std::vector<double> vel_cmds_;
  // state variables: the latest measurement, published to the state handles. All nine come out of
  // one feedback round trip and are decoded together (PHASE2_SPEC 3.5); a joint publishes only the
  // ones its description declares, but every one of them is computed regardless -- skipping a
  // value saves no bus time, and pos_states_ is needed internally whatever the URDF asks for.
  std::vector<double> pos_states_;      // rad, joint frame
  std::vector<double> vel_states_;      // rad/s, joint frame
  std::vector<double> eff_states_;      // N m, a magnitude (D4)
  std::vector<double> cur_states_;      // A, a magnitude (D4)
  std::vector<double> volt_states_;     // V
  std::vector<double> temp_states_;     // deg C
  std::vector<double> load_states_;     // fraction of full PWM, about -1.023 .. +1.023
  std::vector<double> status_states_;   // bitmask 0..255, integer-valued, or NaN for "no reply"
  std::vector<double> torq_states_;     // kg cm, the deprecated alias of eff_states_ (D2)
  // the status byte exactly as feedback() captured it, before anything else could overwrite
  // SCS::Error; the fault-edge block in read() and status_states_ both come from here
  std::vector<u8> status_bytes_;
  // where a joint that on_activate (or the park of on_shutdown/on_error) found outside its
  // position limits is held until it is commanded to a position inside them; NaN for every
  // other joint
  std::vector<double> hold_pos_;
  // which servos answered Ping; absent ones are never put on the bus
  std::vector<bool> present_;
  std::vector<int> read_fails_;
  // consecutive write() cycles whose sync write was REFUSED -- a closed port, or a goal naming an
  // id outside 1..253. Both mean nothing reached the bus and every servo kept the goals of the
  // previous cycle, which on the velocity path means a turning wheel keeps turning, so the ERROR
  // is loud; this counter is what keeps it from being a flood (PHASE3 1.27)
  int write_refusals_ = 0;
  std::vector<u8> last_error_;
  // whether pos_states_ holds a position the servo returned since on_configure, rather than a
  // stand-in (NaN, on_activate's 0.0, or read()'s mirror of the command of an absent servo)
  std::vector<bool> measured_;
  // the continuous position count of each joint, one per joint whether it unwraps or not, and the
  // number of lost-revolution gaps each has been warned about (the throttle counter of 8.5)
  std::vector<PositionUnwrapper> unwrap_;
  std::vector<int> unwrap_gap_warns_;
  // command groups containing only servos that are present, with the joint index each maps to
  std::vector<u8> p_ids_;
  std::vector<size_t> p_js_;
  std::vector<u8> v_ids_;
  std::vector<size_t> v_js_;
  // last non-zero control period, used to pace the goal speed
  double last_period_ = 0.01;
  // One goal record per servo in p_ids_ / v_ids_, refilled by send_commands() every cycle. Unlike
  // the vendored arrays these replaced, they are not rewritten by the send (PHASE3 1.11) -- the
  // wrapper copies into its own scratch -- so the refill is now a choice rather than a correctness
  // requirement. The wheel record carries no acceleration: register 41 is SRAM and is written on
  // its own schedule (PHASE3 1.24), while a position servo's arrives in byte 0 of every record.
  std::vector<GoalPosition> p_goals_;
  std::vector<GoalSpeed> v_goals_;
  // The read group: every joint whose servo is present, in URDF order, and deliberately NOT one of
  // the command groups. A dead id left in a sync-read request costs a whole io_timeout_ms every
  // cycle -- +18.6 ms at 20 ms and +3.56 ms at 5 ms, independent of where in the list it sits and
  // of how many are absent [P1 Q5, P3 Q3] -- while a dead id in a sync WRITE costs three bytes
  // nobody waits for, which is why the write groups keep a dropped servo and this one does not
  // (PHASE3 2.64, F21). The three vectors are parallel: r_ids_[k] is asked for, r_js_[k] is the
  // joint it belongs to and r_blocks_[k] is where its reply lands.
  //
  // uint8_t would be the ServoBus spelling, but INST.h:12 typedefs u8 to unsigned char, so this
  // IS that type and p_ids_ / v_ids_ are left alone.
  std::vector<u8> r_ids_;
  std::vector<size_t> r_js_;
  std::vector<FeedbackBlock> r_blocks_;
  // joint index -> its slot in r_ids_, or kNoSlot when this joint is not in the burst. Replies
  // come back in REQUEST order -- measured with ascending, descending and two shuffled id lists
  // [P1 Q5] -- so slot k is ids[k] and this is an exact mapping rather than a search that read()
  // would otherwise repeat per joint per cycle.
  std::vector<size_t> r_slot_;
  static constexpr size_t kNoSlot = std::numeric_limits<size_t>::max();
  // A drop fired inside read()'s loop. The group is rebuilt after the loop and never during it:
  // the joints after i are still reading r_slot_, and compacting in place would silently re-point
  // them at another servo's sample (PHASE3 2.65, 3.28).
  bool read_group_dirty_ = false;
  // Which transport read() takes. Set by the activation probe and by nothing else -- a mode that
  // flipped mid-run would make the timing unattributable and the tests non-deterministic, and the
  // fault a per-cycle heuristic would fire on is already handled, more cheaply, by the drop policy
  // (PHASE3 2.73, 2.75, R7).
  bool sync_read_active_ = false;
  // Phase 3 item 3, the failed-transaction rate. read_fails_ above is a CONSECUTIVE counter that
  // any success resets, so a bus losing one reply in every other cycle reports nothing at all
  // through it; these are cumulative per joint since on_activate (PHASE3 3.34, R16). Only the
  // real-time path is counted: the two lifecycle reads of an activation would skew a short run and
  // mean nothing in a long one. read_failures_ is what the `bus totals:` tail names per servo;
  // read_attempts_ is that joint's own denominator, kept because 3.34 mandates both its increment
  // sites and F14 pins the absent branch in front of it: it is how many polls a tail count of 8 is
  // 8 out of, which is the first thing a bug hunt off a dirty soak asks (5.18).
  std::vector<uint64_t> read_attempts_;
  std::vector<uint64_t> read_failures_;
  // read() calls since activation. Not the `transactions` of the log line -- this one only answers
  // "did this component ever run a cycle", which is 3.36's second guard (R16).
  uint64_t read_cycles_ = 0;
  // The two aggregates the `bus totals:` line quotes. The denominator is PHASE3 5.14's, exactly:
  // one per WRAPPER READ ENTRY POINT, which is one sync_read_feedback() for the whole bus on the
  // sync path and one read_feedback_one() per polled joint on the per-servo one. Counting cycles
  // on both instead would divide the per-servo arm's denominator by the servo count and multiply
  // its per-million rate by it, and 5.6's two arms -- the sync run and cand_fallback_r1 -- would
  // not be comparable in the README (5.26). On the sync path this is still exactly one per control
  // cycle, which is what 5.15's gate arithmetic assumes (`expected = 100.0 * soak`, and
  // SOAK_MIN_TRANSACTIONS = 30000 as "five minutes at 100 Hz"). A failure is charged once per
  // transaction -- the shared burst once however many frames it lost, a unicast read for its own
  // joint -- so the rate can never exceed a million per million; the per-joint vectors above stay
  // the breakdown the bracketed tail carries (PHASE3 5.14, R16).
  uint64_t read_transactions_ = 0;
  uint64_t read_transaction_failures_ = 0;
  // the largest value any joint's read_fails_ reached since activation, and how many servos left
  // the read cycle: an isolated lost reply and a servo that is going away look identical in a
  // rate, and the bench gate bounds them separately (PHASE3 5.14, 5.15)
  uint32_t worst_consecutive_ = 0;
  uint32_t dropped_count_ = 0;
  bool read_stats_reported_ = false;
};

}  // namespace waveshare_servos

#endif  // WAVESHARE_SERVOS_HPP_
