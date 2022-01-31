#pragma once

#include <Eigen/Dense>
#include <fuse_core/async_sensor_model.h>
#include <ros/ros.h>

#include <bs_common/extrinsics_lookup_online.h>

namespace bs_models {

/**
 * @brief This base class shows the contract between a SlamInitializer class.
 * The goal of this class is to initialize slam with an initial rough trajectory
 * estimate
 *
 */
class SLAMInitializerBase : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_ALIASES_ONLY(SLAMInitializerBase);

  SLAMInitializerBase();

  virtual ~SLAMInitializerBase() = default;

protected:
  /**
   * @brief todo
   */
  void onInit() override {}

  /**
   * @brief todo
   */
  void onStart() override {}

  /**
   * @brief todo
   */
  void onStop() override {}

  /**
   * @brief publish results of initialization to a InitializedPathMsg
   */
  void PublishResults();

  // get access to extrinsics singleton
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();

  ros::Publisher results_publisher_;
  std::vector<Eigen::Matrix4d> trajectory_;
  std::vector<ros::Time> times_;

  bool initialization_complete_{false};
};

} // namespace bs_models