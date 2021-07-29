#pragma once

#include <ros/node_handle.h>

#include <bs_parameters/parameter_base.h>

namespace bs_parameters { namespace models {

/**
 * @brief Defines the set of parameters required by the Unicycle3DIgnition class
 */
struct Unicycle3DIgnitionParams : public ParameterBase {
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  void loadFromROS(const ros::NodeHandle& nh) final {
    //(x, y, yaw, x_vel, y_vel, yaw_vel, x_acc, y_acc).
    //(x, y, z, x_vel, y_vel, z_vel, roll, pitch, yaw,, roll_vel, pitch_vel,
    //yaw_vel, x_acc, y_acc, z_acc)

    nh.getParam("publish_on_startup", publish_on_startup);
    nh.getParam("queue_size", queue_size);
    nh.getParam("reset_service", reset_service);
    nh.getParam("set_pose_service", set_pose_service);
    nh.getParam("set_pose_deprecated_service", set_pose_deprecated_service);
    nh.getParam("topic", topic);

    std::vector<double> sigma_vector;
    if (nh.getParam("initial_sigma", sigma_vector)) {
      if (sigma_vector.size() != 15) {
        throw std::invalid_argument("The supplied initial_sigma parameter must "
                                    "be length 15, but is actually length " +
                                    std::to_string(sigma_vector.size()));
      }
      auto is_sigma_valid = [](const double sigma) {
        return std::isfinite(sigma) && (sigma > 0);
      };
      if (!std::all_of(sigma_vector.begin(), sigma_vector.end(),
                       is_sigma_valid)) {
        throw std::invalid_argument(
            "The supplied initial_sigma parameter must contain valid floating "
            "point values. "
            "NaN, Inf, and values <= 0 are not acceptable.");
      }
      initial_sigma.swap(sigma_vector);
    }

    std::vector<double> state_vector;
    if (nh.getParam("initial_state", state_vector)) {
      if (state_vector.size() != 15) {
        throw std::invalid_argument("The supplied initial_state parameter must "
                                    "be length 8, but is actually length " +
                                    std::to_string(state_vector.size()));
      }
      auto is_state_valid = [](const double state) {
        return std::isfinite(state);
      };
      if (!std::all_of(state_vector.begin(), state_vector.end(),
                       is_state_valid)) {
        throw std::invalid_argument("The supplied initial_state parameter must "
                                    "contain valid floating point values. "
                                    "NaN, Inf, etc are not acceptable.");
      }
      initial_state.swap(state_vector);
    }
  }

  /**
   * @brief Flag indicating if an initial state transaction should be sent on
   * startup, or only in response to a set_pose service call or topic message.
   */
  bool publish_on_startup{true};

  /**
   * @brief The size of the subscriber queue for the set_pose topic
   */
  int queue_size{10};

  /**
   * @brief The name of the reset service to call before sending transactions to
   * the optimizer
   */
  std::string reset_service{"~reset"};

  /**
   * @brief The name of the set_pose service to advertise
   */
  std::string set_pose_service{"~set_pose"};

  /**
   * @brief The name of the deprecated set_pose service without return codes
   */
  std::string set_pose_deprecated_service{"~set_pose_deprecated"};

  /**
   * @brief The topic name for received PoseWithCovarianceStamped messages
   */
  std::string topic{"~set_pose"};

  /**
   * @brief The uncertainty of the initial state value
   *
   * Standard deviations are provided as an 8-dimensional vector in the order:
   *   (x, y, yaw, x_vel, y_vel, yaw_vel, x_acc, y_acc)
   * The covariance matrix is created placing the squared standard deviations
   * along the diagonal of an 8x8 matrix.
   */
  //    std::vector<double> initial_sigma
  //    {1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9};
  std::vector<double> initial_sigma{1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9,
                                    1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9,
                                    1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9};
  //    x y z xd yd zd r p y rd pd yd ax ay az
  //    0 1 2 3  4  5  6 7 8 9  10 11 12 13 14

  /**
   * @brief The initial value of the 8-dimension state vector (x, y, yaw, x_vel,
   * y_vel, yaw_vel, x_acc, y_acc)
   */
  std::vector<double> initial_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

}} // namespace bs_parameters::models
