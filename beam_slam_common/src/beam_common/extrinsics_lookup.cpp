#include <beam_common/extrinsics_lookup.h>

#include <beam_common/utils.h>
#include <beam_utils/log.h>
#include <beam_utils/math.h>

namespace beam_common {

ExtrinsicsLookup& ExtrinsicsLookup::GetInstance() {
  static ExtrinsicsLookup instance;
  return instance;
}

ExtrinsicsLookup::ExtrinsicsLookup() {
  // get parameters from global namespace
  ros::param::get("~imu_frame", imu_frame_);
  ros::param::get("~camera_frame", camera_frame_);
  ros::param::get("~lidar_frame", lidar_frame_);
  ros::param::get("~world_frame", world_frame_);
  ros::param::get("~baselink_frame", baselink_frame_);
  ros::param::get("~static_extrinsics", static_extrinsics_);

  // validate parameters
  if (imu_frame_.empty() || camera_frame_.empty() || lidar_frame_.empty() ||
      baselink_frame_.empty() || baselink_frame_.empty()) {
    BEAM_ERROR(
        "Inputs to ExtrinsicsLookup invalid. You must supply a frame name "
        "for each of the frame types: imu, camera, lidar, baselink, world");
    throw std::invalid_argument{"Inputs to ExtrinsicsLookup invalid."};
  }

  if (baselink_frame_ == imu_frame_ || baselink_frame_ == camera_frame_ ||
      baselink_frame_ == lidar_frame_) {
    // good
  } else {
    BEAM_ERROR("Baselink frame must be equal to one of the sensor frames.");
    throw std::invalid_argument{"Inputs to ExtrinsicsLookup invalid."};
  }
}

bool ExtrinsicsLookup::GetT_CAMERA_IMU(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_IMU_CAMERA_set_) {
    T = beam::InvertTransform(T_IMU_CAMERA_);
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, camera_frame_, imu_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_IMU_CAMERA_ = beam::InvertTransform(T);
    T_IMU_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_IMU_CAMERA(Eigen::Matrix4d& T,
                                       const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_IMU_CAMERA_set_) {
    T = T_IMU_CAMERA_;
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, imu_frame_, camera_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_IMU_CAMERA_ = T;
    T_IMU_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_CAMERA_LIDAR(Eigen::Matrix4d& T,
                                         const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_LIDAR_CAMERA_set_) {
    T = beam::InvertTransform(T_LIDAR_CAMERA_);
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, camera_frame_, lidar_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_LIDAR_CAMERA_ = beam::InvertTransform(T);
    T_LIDAR_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_LIDAR_CAMERA(Eigen::Matrix4d& T,
                                         const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_LIDAR_CAMERA_set_) {
    T = T_LIDAR_CAMERA_;
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, lidar_frame_, camera_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_LIDAR_CAMERA_ = T;
    T_LIDAR_CAMERA_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_IMU_LIDAR(Eigen::Matrix4d& T,
                                      const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_LIDAR_IMU_set_) {
    T = beam::InvertTransform(T_LIDAR_IMU_);
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, imu_frame_, lidar_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_LIDAR_IMU_ = beam::InvertTransform(T);
    T_LIDAR_IMU_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_LIDAR_IMU(Eigen::Matrix4d& T,
                                      const ros::Time& time) {
  // check if already known
  if (static_extrinsics_ && T_LIDAR_IMU_set_) {
    T = T_LIDAR_IMU_;
    return true;
  }

  // get extrinsics
  if (!GetTransform(T, lidar_frame_, imu_frame_, time)) {
    return false;
  }

  // if extrinsics are set, we never have to recalculate this
  if (static_extrinsics_) {
    T_LIDAR_IMU_ = T;
    T_LIDAR_IMU_set_ = true;
  }
  return true;
}

bool ExtrinsicsLookup::GetT_BASELINK_IMU(Eigen::Matrix4d& T,
                                         const ros::Time& time) {
  if (baselink_frame_ == imu_frame_) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (baselink_frame_ == camera_frame_) {
    return GetT_CAMERA_IMU(T, time);
  } else if (baselink_frame_ == lidar_frame_) {
    return GetT_LIDAR_IMU(T, time);
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookup::GetT_IMU_BASELINK(Eigen::Matrix4d& T,
                                         const ros::Time& time) {
  if (baselink_frame_ == imu_frame_) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (baselink_frame_ == camera_frame_) {
    return GetT_IMU_CAMERA(T, time);
  } else if (baselink_frame_ == lidar_frame_) {
    return GetT_IMU_LIDAR(T, time);
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookup::GetT_BASELINK_CAMERA(Eigen::Matrix4d& T,
                                            const ros::Time& time) {
  if (baselink_frame_ == imu_frame_) {
    return GetT_IMU_CAMERA(T, time);
  } else if (baselink_frame_ == camera_frame_) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (baselink_frame_ == lidar_frame_) {
    return GetT_LIDAR_CAMERA(T, time);
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookup::GetT_CAMERA_BASELINK(Eigen::Matrix4d& T,
                                            const ros::Time& time) {
  if (baselink_frame_ == imu_frame_) {
    return GetT_CAMERA_IMU(T, time);
  } else if (baselink_frame_ == camera_frame_) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (baselink_frame_ == lidar_frame_) {
    return GetT_CAMERA_LIDAR(T, time);
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookup::GetT_BASELINK_LIDAR(Eigen::Matrix4d& T,
                                           const ros::Time& time) {
  if (baselink_frame_ == imu_frame_) {
    return GetT_IMU_LIDAR(T, time);
  } else if (baselink_frame_ == camera_frame_) {
    return GetT_CAMERA_LIDAR(T, time);
  } else if (baselink_frame_ == lidar_frame_) {
    T = Eigen::Matrix4d::Identity();
    return true;
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookup::GetT_LIDAR_BASELINK(Eigen::Matrix4d& T,
                                           const ros::Time& time) {
  if (baselink_frame_ == imu_frame_) {
    return GetT_LIDAR_IMU(T, time);
  } else if (baselink_frame_ == camera_frame_) {
    return GetT_LIDAR_CAMERA(T, time);
  } else if (baselink_frame_ == lidar_frame_) {
    T = Eigen::Matrix4d::Identity();
    return true;
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookup::GetT_BASELINK_SENSOR(Eigen::Matrix4d& T,
                                            const std::string& sensor_frame,
                                            const ros::Time& time) {
  if (sensor_frame == baselink_frame_) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (sensor_frame == imu_frame_) {
    return GetT_BASELINK_IMU(T, time);
  } else if (sensor_frame == camera_frame_) {
    return GetT_BASELINK_CAMERA(T, time);
  } else if (sensor_frame == lidar_frame_) {
    return GetT_BASELINK_LIDAR(T, time);
  } else {
    BEAM_WARN(
        "Cannot lookup extrinsics between sensor frame: {} and baselink frame: "
        "{}. Ensure sensor frame ID matches either imu, camera, or lidar "
        "frames and transformation exists at time: {}",
        sensor_frame, baselink_frame_, time.toSec());
    return false;
  }
}

bool ExtrinsicsLookup::GetT_SENSOR_BASELINK(Eigen::Matrix4d& T,
                                            const std::string& sensor_frame,
                                            const ros::Time& time) {
  if (GetT_BASELINK_SENSOR(T, sensor_frame, time)) {
    Eigen::Matrix4d T = beam::InvertTransform(T);
    return true;
  } else {
    // warning thrown by GetT_BASELINK_SENSOR
    return false;
  }
}

bool ExtrinsicsLookup::GetTransform(Eigen::Matrix4d& T,
                                    const std::string& to_frame,
                                    const std::string& from_frame,
                                    const ros::Time& time) {
  tf::StampedTransform TROS;
  try {
    tf_listener_.lookupTransform(to_frame, from_frame, time, TROS);
  } catch (tf::TransformException& ex) {
    if (static_extrinsics_) {
      BEAM_WARN("Cannot lookup static extrinsics between frames: {} , {}",
                to_frame, from_frame);
    } else {
      BEAM_WARN("Cannot lookup dynamic extrinsics for t = %.10f", time.toSec());
    }
    return false;
  }

  ROSStampedTransformToEigenTransform(TROS, T);

  return true;
}

}  // namespace beam_common