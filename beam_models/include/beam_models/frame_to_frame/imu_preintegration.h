#pragma once

#include <queue>

#include <sensor_msgs/Imu.h>
#include <slamtools/preintegrator.h>

#include <beam_constraints/frame_to_frame/imu_state_3d_stamped_transaction.h>
#include <beam_models/frame_to_frame/imu_state.h>

namespace beam_models {
namespace frame_to_frame {

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
    double gravitational_acceleration{9.80665};
    double prior_noise{1e-9};
    Eigen::Matrix3d cov_gyro_noise;
    Eigen::Matrix3d cov_accel_noise;
    Eigen::Matrix3d cov_gyro_bias;
    Eigen::Matrix3d cov_accel_bias;
    std::string source{"IMUPREINTEGRATION"};
  };

  /**
   * @brief IMU data type derived from Slamtools
   * @param t_ros ros time of imu data
   */
  struct ImuData : public IMUData {
    ros::Time t_ros;

    ImuData() = default;

    /**
     * @brief Constructor
     * @param msg sensor data
     */
    ImuData(const sensor_msgs::Imu::ConstPtr& msg) {
      t_ros = msg->header.stamp;
      t = msg->header.stamp.toSec();
      w[0] = msg->angular_velocity.x;
      w[1] = msg->angular_velocity.y;
      w[2] = msg->angular_velocity.z;
      a[0] = msg->linear_acceleration.x;
      a[1] = msg->linear_acceleration.y;
      a[2] = msg->linear_acceleration.z;
    }
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
  void PopulateBuffer(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief populate imu buffer with imu data
   */
  void PopulateBuffer(const ImuData& imu_data);

  /**
   * @brief sets the initial IMU state with respect to world frame
   * @param t_start time of initial IMU state
   * @param orientation orientation of initial IMU state
   * @param position position of initial IMU state
   * @param velocity velocity of initial IMU state
   */
  void SetStart(
      const ros::Time& t_start,
      fuse_variables::Orientation3DStamped::SharedPtr orientation = nullptr,
      fuse_variables::Position3DStamped::SharedPtr position = nullptr,
      fuse_variables::VelocityLinear3DStamped::SharedPtr velocity = nullptr);

  /**
   * @brief predicts new IMU state from imu preintegration
   * @param pre_integrator Preintegrator class containing imu data between new
   * and current IMU states
   * @param imu_state_curr current IMU state
   * @return ImuState
   */
  ImuState PredictState(const PreIntegrator& pre_integrator,
                        const ImuState& imu_state_curr);

  /**
   * @brief calculates relative change between new and previous key frames
   * @param imu_state_new new ImuState
   * @return delta vector in order [del_q, del_p, del_v, del_bg, ba]
   */
  Eigen::Matrix<double, 16, 1> CalculateRelativeChange(
      const ImuState& imu_state_new);

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
   * @param orientation orientation of new key frame from VIO or LIO
   * @param position position of new key frame from VIO or LIO
   * @return transaction
   */
  beam_constraints::frame_to_frame::ImuState3DStampedTransaction
  RegisterNewImuPreintegratedFactor(
      const ros::Time& t_now,
      fuse_variables::Orientation3DStamped::SharedPtr orientation = nullptr,
      fuse_variables::Position3DStamped::SharedPtr position = nullptr);

 private:
  /**
   * @brief sets Preintegrator class parameters
   */
  void SetPreintegrator();

  /**
   * @brief calls Preintegrator reset() and clears stored imu data
   */
  void ResetPreintegrator();

  Params params_;            // class parameters
  Eigen::Vector3d g_;        // gravity vector
  bool first_window_{true};  // flag for first window between key frames

  ImuState imu_state_i_;                 // current key frame
  ImuState imu_state_k_;                 // intermediate frame
  PreIntegrator pre_integrator_ij;       // preintegrate between key frames
  std::queue<ImuData> imu_data_buffer_;  // store imu data
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};  // zero gyroscope bias
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};  // zero accleration bias
};

}  // namespace frame_to_frame
}  // namespace beam_models
