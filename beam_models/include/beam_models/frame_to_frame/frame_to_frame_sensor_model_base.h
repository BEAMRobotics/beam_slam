#pragma once

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/uuid.h>
#include <fuse_models/common/throttled_callback.h>

#include <beam_constraints/frame_to_frame/frame_to_frame_transaction_base.h>
#include <beam_models/frame_initializers/frame_initializers.h>
#include <beam_parameters/models/frame_to_frame_parameter_base.h>

namespace beam_models { namespace frame_to_frame {

/**
 * This class is inherited by each of the different frame_to_frame sensor
 * models.
 * @tparam T sensor data type being subscribed to
 * @tparam Parameter type. Type must be derived from
 * beam_parameters::ParameterBase and must contain the following variabels:
 *  (1) frame_initializer_type
 *  (2) frame_initializer_info
 *  (3) sensor_frame
 *  (4) subscriber_topic
 */
template <typename SubscriberMsgType, typename ParamType,
          typename TransactionType>
class FrameToFrameSensorModelBase : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(FrameToFrameSensorModelBase<
                        SubscriberMsgType, ParamType, TransactionType>);

  /**
   * @brief This constructor loads variables, and initializes the
   * frame_initializer object
   */
  FrameToFrameSensorModelBase()
      : fuse_core::AsyncSensorModel(1),
        device_id_(fuse_core::uuid::NIL),
        throttled_callback_(std::bind(&FrameToFrameSensorModelBase::process,
                                      this, std::placeholders::_1)) {}

  ~FrameToFrameSensorModelBase() override = default;

protected:
  /**
   * @brief virtual function that needs to be overridden for each derived class
   */
  virtual TransactionType GenerateTransaction(
      const boost::shared_ptr<SubscriberMsgType const>& msg) = 0;

  /**
   * @brief This initiates all variables common to derived classes. This needs
   * to be called from the derived class onInit() function so that we have
   * access to the node handle
   * @param nh derived class nodehandle. We specific params in the yaml with the
   * namespace of the derived class, therefore this has to be the node handle of
   * the derived class (e.g., ScanMatcher3D)
   */
  void InitiateBaseClass(const ros::NodeHandle& nh) {
    // Read settings from the parameter sever
    device_id_ = fuse_variables::loadDeviceId(nh);
    base_params_ = std::make_shared<ParamType>();
    base_params_->loadFrameToFrameDefaultParams(nh);

    // init frame initializer
    if (base_params_->frame_initializer_type == "ODOMETRY") {
      frame_initializer_ =
          std::make_unique<frame_initializers::OdometryFrameInitializer>(
              base_params_->frame_initializer_info, 100,
              base_params_->sensor_frame, true, 30);
    } else if (base_params_->frame_initializer_type == "POSEFILE") {
      frame_initializer_ =
          std::make_unique<frame_initializers::PoseFileFrameInitializer>(
              base_params_->frame_initializer_info, base_params_->sensor_frame);
    } else {
      const std::string error =
          "frame_initializer_type invalid. Options: ODOMETRY, POSEFILE";
      ROS_FATAL_STREAM(error);
      throw std::runtime_error(error);
    }
  }

  void process(const boost::shared_ptr<SubscriberMsgType const>& msg) {
    TransactionType new_transaction = GenerateTransaction(msg);
    if (new_transaction.GetTransaction() != nullptr) {
      ROS_DEBUG("Sending transaction.");
      sendTransaction(new_transaction.GetTransaction());
    }
  }

  /**
   * @brief Subscribe to the input topic to start sending transactions to the
   * optimizer
   */
  void onStart() override {
    subscriber_ = node_handle_.subscribe(base_params_->subscriber_topic, 10,
                                         &ThrottledCallback::callback,
                                         &throttled_callback_);
  };

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the
   * optimizer
   */
  void onStop() override { subscriber_.shutdown(); };

  fuse_core::UUID device_id_; //!< The UUID of this device
  std::shared_ptr<beam_parameters::models::FrameToFrameParameterBase>
      base_params_;
  std::unique_ptr<frame_initializers::FrameInitializerBase> frame_initializer_;
  ros::Subscriber subscriber_;

  using ThrottledCallback =
      fuse_models::common::ThrottledCallback<SubscriberMsgType>;
  ThrottledCallback throttled_callback_;
};

}} // namespace beam_models::frame_to_frame
