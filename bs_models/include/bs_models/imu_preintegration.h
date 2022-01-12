#pragma once

#include <queue>

#include <bs_common/bs_msgs.h>
#include <bs_common/imu_state.h>
#include <bs_common/preintegrator.h>
#include <bs_constraints/relative_pose/imu_state_3d_stamped_transaction.h>
#include <sensor_msgs/Imu.h>

namespace bs_models {

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
   * @param R_WORLD_IMU orientation of initial IMU state (if null set to id.)
   * @param t_WORLD_IMU position of initial IMU state (if null set to zero)
   * @param velocity velocity of initial IMU state (if null set to zero)
   */
  void SetStart(
      const ros::Time& t_start,
      fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU = nullptr,
      fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU = nullptr,
      fuse_variables::VelocityLinear3DStamped::SharedPtr velocity = nullptr);

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
   * @brief Gets pose of IMU with respect to world frame
   * @param t_now time at which to get pose
   * @param T_WORLD_IMU reference to pose matrix to fill in
   * @return true if successful
   */
  bool GetPose(
      Eigen::Matrix4d& T_WORLD_IMU, const ros::Time& t_now,
      std::shared_ptr<Eigen::Matrix<double, 6, 6>> covariance = nullptr);

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
  void UpdateGraph(fuse_core::Graph::SharedPtr graph_msg);

  /**
   * @brief Estimates inertial parameters given an initial path and imu messages
   */
  static void EstimateParameters(
      const bs_common::InitializedPathMsg& path,
      const std::queue<sensor_msgs::Imu>& imu_buffer,
      const bs_models::ImuPreintegration::Params& params,
      Eigen::Vector3d& gravity, Eigen::Vector3d& bg, Eigen::Vector3d& ba,
      std::vector<Eigen::Vector3d>& velocities, double& scale);

private:
  /**
   * @brief Checks parameters
   */
  void CheckParameters();

  /**
   * @brief Sets Preintegrator class parameters
   */
  void SetPreintegrator();

  /**
   * @brief Calls Preintegrator reset() and clears stored imu data
   */
  void ResetPreintegrator(const ros::Time& t_now);

  Params params_;           // class parameters
  bool first_window_{true}; // flag for first window between key frames

  bs_common::ImuState imu_state_i_;           // current key frame
  bs_common::ImuState imu_state_k_;           // intermediate frame
  bs_common::PreIntegrator pre_integrator_ij; // preintegrate between key frames
  bs_common::PreIntegrator
      pre_integrator_kj; // preintegrate from intermediate frame
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()}; // zero gyroscope bias
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()}; // zero acceleration bias
};

} // namespace bs_models
