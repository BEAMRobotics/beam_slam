#include <bs_models/frame_initializers/internal_frame_initializer.h>
#include <gtest/gtest.h>

TEST(InternalFrameInitializer) {
  InternalFrameInitializer fi =
      bs_models::InternalFrameInitializer::GetInstance();
}