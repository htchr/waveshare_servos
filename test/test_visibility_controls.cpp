// Defines the guard cpplint gives a consumer's own visibility_controls.h. waveshare_servos.hpp
// must still compile, so the installed header needs a package-namespaced guard.
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
