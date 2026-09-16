// A package that uses waveshare_servos can have an include/visibility_controls.h of its own, and
// cpplint gives that header the include guard VISIBILITY_CONTROLS_H_. Define that guard here, as
// such a header does when it is included first: waveshare_servos.hpp must still get its
// WAVESHARE_SERVOS_* macros, so the installed visibility_controls.h keeps a guard in the package's
// own namespace. With a colliding guard this file does not compile.
#define VISIBILITY_CONTROLS_H_

#include <gmock/gmock.h>

#include <type_traits>

#include "hardware_interface/system_interface.hpp"
#include "waveshare_servos.hpp"

TEST(WaveshareServosHeaders, public_header_compiles_after_a_consumer_visibility_controls_header)
{
  EXPECT_TRUE(
    (std::is_base_of<hardware_interface::SystemInterface,
    waveshare_servos::WaveshareServos>::value));
}
