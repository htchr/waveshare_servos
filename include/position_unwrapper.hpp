// A continuous, multi-turn position count built from a wrapping servo register (PHASE2_SPEC 8.1).
//
// Driver-owned, header-only and pure: integer arithmetic over one tick sequence, with no
// ros2_control, no rclcpp and none of the vendored SCServo headers, so it is testable with no bus
// and no motors.
//
// One rule covers both a wrapping register and a hypothetical multi-turn firmware, with no mode
// switch and no probe: every delta between two accepted samples is reduced into a half-open half
// revolution and added to the count. A register that wraps 4095 -> 12 therefore advances by 13
// ticks, and a firmware that already counts turns is simply followed.
//
// Correctness envelope: the rule returns the true delta iff the servo turned less than half a
// revolution between two ACCEPTED samples (2048 ticks = pi rad at 4096 steps). Speed alone cannot
// break it -- the driver's 9.2039 rad/s ceiling is 60 ticks per 10 ms cycle, 34x of headroom -- so
// aliasing needs a sample GAP of |omega| * dt >= pi. The failure is silent and permanent (the count
// is short by whole revolutions), which is why the driver warns about a gap long enough to have
// caused one (PHASE2_SPEC 8.5) and why a_gap_longer_than_half_a_revolution_aliases pins it.
//
// Rejected alternatives, so they are not reinvented: re-seeding after a gap makes the error the
// full travel instead of bounding it to whole revolutions (strictly worse); de-aliasing from
// ReadSpeed turns a pure function into one that depends on elapsed time and a second register.

#ifndef POSITION_UNWRAPPER_HPP_
#define POSITION_UNWRAPPER_HPP_

#include <cstdint>

namespace waveshare_servos
{

class PositionUnwrapper
{
public:
  // The constructor clamps exactly as the setter below does; PHASE2_SPEC 8.1's class block stores
  // the argument as given, and the difference (a declared deviation) is that a PositionUnwrapper(0)
  // reduces deltas at 2 steps instead of freezing the count. Unreachable from the driver, which
  // validates encoder_steps to an even value in [2, 32768] before building these, and pinned by
  // steps_below_two_freezes_the_count so the two entry points to steps_ cannot drift apart.
  explicit PositionUnwrapper(int32_t steps_per_revolution = 4096)
  : steps_(steps_per_revolution < 2 ? 2 : steps_per_revolution) {}

  void set_steps_per_revolution(int32_t steps) {steps_ = steps < 2 ? 2 : steps;}
  int32_t steps_per_revolution() const {return steps_;}

  void reset() {ticks_ = 0; last_raw_ = 0; seeded_ = false; bridged_ = false;}
  bool seeded() const {return seeded_;}
  bool bridged() const {return bridged_;}       // the last update() continued an existing count
  int64_t ticks() const {return ticks_;}
  int32_t last_raw() const {return last_raw_;}

  // The first call after construction or reset() seeds the count at `raw`, so the first sample of
  // an activation is exactly the value the driver reported before unwrapping existed. int64_t at
  // the driver's ceiling of 6000 ticks/s saturates in about 49 million years.
  int64_t update(int32_t raw)
  {
    if (!seeded_) {
      ticks_ = raw;
      last_raw_ = raw;
      seeded_ = true;
      bridged_ = false;
      return ticks_;
    }
    ticks_ += wrapped_delta(static_cast<int64_t>(raw) - static_cast<int64_t>(last_raw_), steps_);
    last_raw_ = raw;
    bridged_ = true;
    return ticks_;
  }

  /// `delta` reduced into [-steps/2, +steps/2); exactly half a revolution reads as backward.
  /// Total: a `steps` below 2 freezes the count instead of dividing by zero on the real-time path.
  static int64_t wrapped_delta(int64_t delta, int32_t steps)
  {
    if (steps < 2) {
      return 0;
    }
    const int64_t s = steps;
    int64_t r = delta % s;          // C++ truncates toward zero, so r is in (-s, s)
    if (r < 0) {
      r += s;                       // r is now in [0, s)
    }
    // `2 * r >= s` rather than `r >= s / 2`: identical for every even step count, which is all the
    // driver allows (on_init requires an even encoder_steps), but an odd one truncates -- at 4095
    // steps half a revolution is 2047.5, and `r >= s / 2` would send 2047 the long way round, out
    // of the interval this comment promises. 2 * r cannot overflow: r < s <= INT32_MAX.
    if (2 * r >= s) {
      r -= s;                       // r is now in [-s/2, s/2)
    }
    return r;
  }

private:
  int32_t steps_ = 4096;
  int64_t ticks_ = 0;
  int32_t last_raw_ = 0;
  bool seeded_ = false;
  bool bridged_ = false;
};

}  // namespace waveshare_servos

#endif  // POSITION_UNWRAPPER_HPP_
