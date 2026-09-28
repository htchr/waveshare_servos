// Multi-turn tick count from the wrapping position register. Correct while the servo turns less
// than half a turn between two samples. See docs/configuration.md, "Multi-turn wheel position".

#ifndef POSITION_UNWRAPPER_HPP_
#define POSITION_UNWRAPPER_HPP_

#include <cstdint>

namespace waveshare_servos
{

class PositionUnwrapper
{
public:
  // Clamps steps below 2 to 2, as set_steps_per_revolution() does. The driver always passes an
  // even encoder_steps in [2, 32768].
  explicit PositionUnwrapper(int32_t steps_per_revolution = 4096)
  : steps_(steps_per_revolution < 2 ? 2 : steps_per_revolution) {}

  void set_steps_per_revolution(int32_t steps) {steps_ = steps < 2 ? 2 : steps;}
  int32_t steps_per_revolution() const {return steps_;}

  void reset() {ticks_ = 0; last_raw_ = 0; seeded_ = false; bridged_ = false;}
  bool seeded() const {return seeded_;}
  bool bridged() const {return bridged_;}       // the last update() continued an existing count
  int64_t ticks() const {return ticks_;}
  int32_t last_raw() const {return last_raw_;}

  // The first call after construction or reset() seeds the count at `raw`. int64_t lasts about
  // 9 million years even at the 32767 ticks/s max_speed cap.
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
    // `2 * r >= s`, not `r >= s / 2`: also right for an odd step count, where s / 2 truncates.
    // 2 * r cannot overflow: r < s <= INT32_MAX.
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
