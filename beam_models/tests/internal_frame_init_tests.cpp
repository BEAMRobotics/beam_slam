#include <beam_models/frame_initializers/internal_frame_initializer.h>
#include <gtest/gtest.h>

TEST(InternalFrameInitializer) {
  InternalFrameInitializer fi =
      beam_models::frame_initializers::InternalFrameInitializer::GetInstance();
}