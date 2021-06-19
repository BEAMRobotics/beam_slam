#pragma once

#include <queue>

#include <sensor_msgs/Imu.h>
#include <beam_constraints/frame_to_frame/imu_state_3d_stamped_transaction.h>
#include <beam_models/frame_to_frame/imu_state.h>
#include <beam_common/preintegrator.h>

#define GRAVITY 9.80655

namespace beam_models { namespace frame_to_frame {

template <typename ConstraintType, typename PriorType>
using TransactionBase =
    beam_constraints::frame_to_frame::FrameToFrameTransactionBase<
        ConstraintType, PriorType>;

/**
 * @brief this class can be used to estimate orientation, position, and
 * velocity of an imu with respect to the world frame using the methods
 * outlined in DOI: 10.15607/RSS.2015.XI.006. This class also estimates imu
 * gyroscope and accleration bias by concidering them to be slowly time-varying
 * quantities. Key frames calculated from imu preintegration can be adjusted
 * with estimates of pose outside of this class, such as those obtained from VIO
 * or LIO
 */
class ImuPreintegration {
public:
  /**
   * @param gravitational_acceleration gravitational acceleration in [m/sec^2]
   * @param prior_noise noise assumed for prior covariance
   * @param cov_gyro_noise angular velocity covariance [REQUIRED]
   * @param cov_accel_noise linear accleration covariance [REQUIRED]
   * @param cov_gyro_bias gyroscope bias covariance [REQUIRED]
   * @param cov_accel_bias accleration bias covariance [REQUIRED]
   * @param source sensor model source
   */
  struct Params {
    double prior_noise{1e-9};
    Eigen::Matrix3d cov_gyro_noise{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d cov_accel_noise{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d cov_gyro_bias{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d cov_accel_bias{Eigen::Matrix3d::Identity()};
    std::string source{"IMUPREINTEGRATION"};
  };

  /**
   * @brief Constructor
   * @param params all input params required. See struct above
   */
  ImuPreintegration(const Params& params);

  /**
   * @brief Constructor
   * @param params all input params required. See struct above
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
   * @brief clear imu buffer
   */
  void ClearBuffer();

  /**
   * @brief populate imu buffer with imu data collected by sensor data
   */
  void AddToBuffer(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief populate imu buffer with imu data
   */
  void AddToBuffer(const beam_common::IMUData& imu_data);

  /**
   * @brief sets the initial IMU state with respect to world frame
   * @param t_start time of initial IMU state
   * @param R_WORLD_IMU orientation of initial IMU state (if null, is set to identity)
   * @param t_WORLD_IMU position of initial IMU state (if null, is set to zero)
   * @param velocity velocity of initial IMU state (if null set to zero)
   */
  void SetStart(
      const ros::Time& t_start,
      fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU = nullptr,
      fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU = nullptr,
      fuse_variables::VelocityLinear3DStamped::SharedPtr velocity = nullptr);

  /**
   * @brief predicts new IMU state from imu preintegration
   * @param pre_integrator Preintegrator class containing imu data between new
   * and current IMU states
   * @param imu_state_curr current IMU state
   * @return ImuState
   */
  ImuState PredictState(const beam_common::PreIntegrator& pre_integrator,
                        const ImuState& imu_state_curr);

  /**
   * @brief calculates relative change between new and previous key frames
   * @param imu_state_new new ImuState
   * @return delta vector in order [del_q, del_p, del_v, del_bg, ba]
   */
  Eigen::Matrix<double, 16, 1>
      CalculateRelativeChange(const ImuState& imu_state_new);

  /**
   * @brief gets current IMU state, which is the last registered key frame
   * @return ImuState
   */
  ImuState GetImuState() const { return imu_state_i_; }

  /**
   * @brief gets pose of imu with respect to world frame
   * @param t_now time at which to get pose
   * @return pose
   */
  Eigen::Matrix4d GetPose(const ros::Time& t_now);

  /**
   * @brief registers new transaction between key frames
   * @param t_now time at which to set new key frame
   * @param R_WORLD_IMU orientation of new key frame from VIO or LIO (if null, imu will predict)
   * @param t_WORLD_IMU position of new key frame from VIO or LIO (if null, imu will predict)
   * @return transaction
   */
  beam_constraints::frame_to_frame::ImuState3DStampedTransaction
      RegisterNewImuPreintegratedFactor(
          const ros::Time& t_now,
          fuse_variables::Orientation3DStamped::SharedPtr R_WORLD_IMU = nullptr,
          fuse_variables::Position3DStamped::SharedPtr t_WORLD_IMU = nullptr);

private:
  /**
   * @brief sets Preintegrator class parameters
   */
  void SetPreintegrator();

  /**
   * @brief calls Preintegrator reset() and clears stored imu data
   */
  void ResetPreintegrator();

  /**
   * @brief checks to see if requested time preceeds IMU messages in buffer.
   * Throws fatal run time error if this is true
   * @param t_now requested time
   */
  void CheckTime(const ros::Time& t_now);

  Params params_;           // class parameters
  Eigen::Vector3d g_;       // gravity vector
  bool first_window_{true}; // flag for first window between key frames

  ImuState imu_state_i_;                // current key frame
  ImuState imu_state_k_;                // intermediate frame
  beam_common::PreIntegrator pre_integrator_ij;      // preintegrate between key frames
  std::queue<beam_common::IMUData> imu_data_buffer_; // store imu data
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()}; // zero gyroscope bias
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()}; // zero accleration bias
};

}} // namespace beam_models::frame_to_frame
