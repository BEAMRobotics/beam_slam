#pragma once

#include <queue>

#include <bs_common/bs_msgs.h>
#include <bs_common/imu_state.h>
#include <bs_common/preintegrator.h>
#include <bs_constraints/relative_pose/imu_state_3d_stamped_transaction.h>
#include <sensor_msgs/Imu.h>

namespace bs_models {

using PoseWithCovariance =
    std::pair<Eigen::Matrix4d, Eigen::Matrix<double, 6, 6>>;
/**
 * @brief This class can be used to estimate orientation, position, and
 * velocity of an IMU with respect to the world frame using the methods
 * outlined in DOI: 10.15607/RSS.2015.XI.006. This class also estimates IMU
 * gyroscope and acceleration bias by concidering them to be slowly time-varying
 * quantities. Key frames calculated from IMU preintegration can be adjusted
 * with estimates of pose outside of this class, such as those obtained from VIO
 * or LIO
 */
class ImuPreintegration {
public:
  /**
   * @param cov_prior_noise noise assumed for prior covariance on IMU state
   * @param cov_gyro_noise angular velocity covariance
   * @param cov_accel_noise linear acceleration covariance
   * @param cov_gyro_bias gyroscope bias covariance
   * @param cov_accel_bias acceleration bias covariance
   * @param source sensor model source
   */
  struct Params {
    double cov_prior_noise{1e-9};
    Eigen::Matrix3d cov_gyro_noise{Eigen::Matrix3d::Identity() * 1e-4};
    Eigen::Matrix3d cov_accel_noise{Eigen::Matrix3d::Identity() * 1e-3};
    Eigen::Matrix3d cov_gyro_bias{Eigen::Matrix3d::Identity() * 1e-6};
    Eigen::Matrix3d cov_accel_bias{Eigen::Matrix3d::Identity() * 1e-4};
    std::string source{"IMUPREINTEGRATION"};

    bool LoadFromJSON(const std::string& path);
  };

  /**
   * @brief Constructor
   * @param params all input params optional. See struct above
   */
  ImuPreintegration(const Params& params);

  /**
   * @brief Constructor
   * @param params all input params optional. See struct above
   * @param init_bg initial gyroscope bias
   * @param init_ba initial acceleration bias
   */
  ImuPreintegration(const Params& params, const Eigen::Vector3d& init_bg,
                    const Eigen::Vector3d& init_ba);

  /**
   * @brief Destructor
   */
  ~ImuPreintegration() = default;

  /**
   * @brief Clears all data from preintegrators and resets them
   */
  void Clear();

  /**
   * @brief Populate IMU buffer with IMU data from raw sensor data
   */
  void AddToBuffer(const sensor_msgs::Imu& msg);

  /**
   * @brief Populate IMU buffer with IMU data
   */
  void AddToBuffer(const bs_common::IMUData& imu_data);

  /**
   * @brief Sets the initial IMU state with respect to world frame
   * @param t_start time of initial IMU state
   * @param R_WORLD_IMU orientation of initial IMU state (if null set to I)
   * @param t_WORLD_IMU position of initial IMU state (if null set to zero)
   * @param velocity velocity of initial IMU state (if null set to zero)
   */
  void SetStart(
      const ros::Time& t_start,
      fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU = nullptr,
      fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU = nullptr,
      fuse_variables::VelocityLinear3DStamped::SharedPtr velocity = nullptr);

  /**
   * @brief Gets the current pose estimate wrt the graph
   * @param t_now to get pose estimate for
   * @return [T_WORLD_IMU, cov]
   */
  PoseWithCovariance GetPose(const ros::Time& t_now);

  /**
   * @brief Gets the relative motion between timestamps within the current
   * window
   * @param t1 first timestamp
   * @param t2 second timestamp
   * @return [T_IMUSTATE1_IMUSTATE2, cov]
   */
  PoseWithCovariance GetRelativeMotion(const ros::Time& t1,
                                       const ros::Time& t2);

  /**
   * @brief Predicts new IMU state from imu preintegration
   * @param pre_integrator preintegrator class containing IMU data between new
   * and current IMU states
   * @param imu_state_curr current IMU state
   * @param t_now time at which to stamp new IMU state if specified
   * @return ImuState
   */
  bs_common::ImuState
      PredictState(const bs_common::PreIntegrator& pre_integrator,
                   const bs_common::ImuState& imu_state_curr,
                   const ros::Time& t_now = ros::Time(0));

  /**
   * @brief Gets current IMU state, which is the last registered key frame
   * @return ImuState
   */
  bs_common::ImuState GetImuState() const { return imu_state_i_; }

  /**
   * @brief Registers new transaction between key frames
   * @param t_now time at which to set new key frame
   * @param R_WORLD_IMU orientation of new key frame from VIO or LIO (if null,
   * imu will predict)
   * @param t_WORLD_IMU position of new key frame from VIO or LIO (if null, imu
   * will predict)
   * @return transaction if successful. If not, nullptr is returned
   */
  fuse_core::Transaction::SharedPtr RegisterNewImuPreintegratedFactor(
      const ros::Time& t_now,
      fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU = nullptr,
      fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU = nullptr,
      fuse_variables::VelocityLinear3DStamped::SharedPtr velocity = nullptr);

  /**
   * @brief Updates current graph copy
   * @param graph_msg graph to update with
   */
  void UpdateGraph(fuse_core::Graph::ConstSharedPtr graph_msg);

  /**
   * @brief Creates a string representation of the data in the buffer
   */
  std::string PrintBuffer();

private:
  /**
   * @brief Checks parameters
   */
  void CheckParameters();

  /**
   * @brief Sets Preintegrator class parameters
   */
  void SetPreintegrator();

  Params params_;           // class parameters
  bool first_window_{true}; // flag for first window between key frames

  bs_common::ImuState imu_state_i_; // current key frame
  bs_common::ImuState imu_state_k_; // intermediate frame
  bs_common::PreIntegrator
      pre_integrator_ij_; // preintegrate between key frames
  bs_common::PreIntegrator
      pre_integrator_kj_; // preintegrate between every frame
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()}; // zero gyroscope bias
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()}; // zero acceleration bias
  std::map<uint64_t, bs_common::ImuState>
      window_states_; // state velocities in the window
};

} // namespace bs_models
