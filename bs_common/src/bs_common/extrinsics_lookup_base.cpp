#include <bs_common/extrinsics_lookup_base.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <bs_common/utils.h>
#include <beam_utils/log.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/math.h>
#include <beam_utils/time.h>

namespace bs_common {

ExtrinsicsLookupBase::ExtrinsicsLookupBase(const FrameIds& frame_ids)
    : frame_ids_(frame_ids) {
  // validate parameters
  if (frame_ids_.imu.empty() || frame_ids_.camera.empty() ||
      frame_ids_.lidar.empty() || frame_ids_.baselink.empty() ||
      frame_ids_.world.empty()) {
    BEAM_ERROR(
        "Inputs to ExtrinsicsLookupBase invalid. You must supply a frame name "
        "for each of the frame types: imu, camera, lidar, baselink, world");
    throw std::invalid_argument{"Inputs to ExtrinsicsLookupBase invalid."};
  }

  if (!IsSensorFrameIdValid(frame_ids_.baselink)) {
    BEAM_ERROR("Baselink frame must be equal to one of the sensor frames.");
    throw std::invalid_argument{"Inputs to ExtrinsicsLookupBase invalid."};
  }
}

ExtrinsicsLookupBase::ExtrinsicsLookupBase(
    const FrameIds& frame_ids, const std::string& extrinsics_filepath) {
  if (!boost::filesystem::exists(extrinsics_filepath)) {
    BEAM_ERROR("Invalid extrinsics file path, exiting. Input: {}",
               extrinsics_filepath);
    throw std::runtime_error{"Invalid extrinsics filepath"};
  }

  if (!beam::HasExtension(extrinsics_filepath, ".json")) {
    BEAM_ERROR(
        "Invalid file extension for extrinsics, must be json. Exiting. Input "
        "{}",
        extrinsics_filepath);
    throw std::runtime_error{"Invalid extrinsics file extension."};
  }

  BEAM_INFO("Loading extrinsics from: {}", extrinsics_filepath);

  // load general data
  nlohmann::json J;
  std::ifstream file(extrinsics_filepath);
  file >> J;
  for (auto calib : J["calibrations"]) {
    Eigen::Matrix4d::Identity();
    std::vector<double> v = calib["transform"];
    Eigen::Matrix4d T = beam::VectorToEigenTransform(v);
    if (!SetTransform(T, calib["to_frame"], calib["from_frame"])) {
      BEAM_WARN(
          "Cannot set calibration from {} to {} since at least one frame does "
          "not correspond to the sensor frame ids provided to "
          "ExtrinsicsLookupBase. Skipping this calibration.",
          calib["to_frame"], calib["from_frame"]);
    }
  }
}

void ExtrinsicsLookupBase::SaveToJson(const std::string& save_filename) {
  // check input
  boost::filesystem::path path(save_filename);
  path.remove_filename();
  if (!boost::filesystem::exists(path)) {
    BEAM_ERROR(
        "Invalid save path path does not exist, not outputting to json. Input: "
        "{}",
        save_filename);
  }

  if (!beam::HasExtension(save_filename, ".json")) {
    BEAM_ERROR(
        "Invalid file extension for extrinsics, must be json. Exiting. Input "
        "{}",
        save_filename);
    return;
  }

  // create output json and add general info (format from beam_calibration
  // extrinsics)
  nlohmann::json J_out;
  J_out["type"] = "extrinsic_calibration";
  J_out["date"] = beam::ConvertTimeToDate(std::chrono::system_clock::now());
  J_out["method"] = "ExtrinsicsLookupBase";

  // add calibrations
  nlohmann::json J_calibrations = nlohmann::json::array();

  nlohmann::json J_lidar_imu;
  J_lidar_imu["to_frame"] = frame_ids_.lidar;
  J_lidar_imu["from_frame"] = frame_ids_.imu;
  J_lidar_imu["transform"] = beam::EigenTransformToVector(T_LIDAR_IMU_);
  J_calibrations.push_back(J_lidar_imu);

  nlohmann::json J_lidar_camera;
  J_lidar_imu["to_frame"] = frame_ids_.lidar;
  J_lidar_imu["from_frame"] = frame_ids_.camera;
  J_lidar_imu["transform"] = beam::EigenTransformToVector(T_LIDAR_CAMERA_);
  J_calibrations.push_back(J_lidar_camera);

  nlohmann::json J_imu_camera;
  J_lidar_imu["to_frame"] = frame_ids_.imu;
  J_lidar_imu["from_frame"] = frame_ids_.camera;
  J_lidar_imu["transform"] = beam::EigenTransformToVector(T_IMU_CAMERA_);
  J_calibrations.push_back(J_imu_camera);

  J_out["calibrations"] = J_calibrations;

  // output to file
  std::ofstream file(save_filename);
  file << std::setw(4) << J_out << std::endl;
}

bool ExtrinsicsLookupBase::GetT_CAMERA_IMU(Eigen::Matrix4d& T) {
  // check if already known
  if (T_IMU_CAMERA_set_) {
    T = beam::InvertTransform(T_IMU_CAMERA_);
    return true;
  }
  return false;
}

bool ExtrinsicsLookupBase::GetT_IMU_CAMERA(Eigen::Matrix4d& T) {
  // check if already known
  if (T_IMU_CAMERA_set_) {
    T = T_IMU_CAMERA_;
    return true;
  }
  return false;
}

bool ExtrinsicsLookupBase::GetT_CAMERA_LIDAR(Eigen::Matrix4d& T) {
  // check if already known
  if (T_LIDAR_CAMERA_set_) {
    T = beam::InvertTransform(T_LIDAR_CAMERA_);
    return true;
  }
  return false;
}

bool ExtrinsicsLookupBase::GetT_LIDAR_CAMERA(Eigen::Matrix4d& T) {
  // check if already known
  if (T_LIDAR_CAMERA_set_) {
    T = T_LIDAR_CAMERA_;
    return true;
  }
  return false;
}

bool ExtrinsicsLookupBase::GetT_IMU_LIDAR(Eigen::Matrix4d& T) {
  // check if already known
  if (T_LIDAR_IMU_set_) {
    T = beam::InvertTransform(T_LIDAR_IMU_);
    return true;
  }
  return false;
}

bool ExtrinsicsLookupBase::GetT_LIDAR_IMU(Eigen::Matrix4d& T) {
  // check if already known
  if (T_LIDAR_IMU_set_) {
    T = T_LIDAR_IMU_;
    return true;
  }
  return false;
}

bool ExtrinsicsLookupBase::GetT_BASELINK_IMU(Eigen::Matrix4d& T) {
  if (frame_ids_.baselink == frame_ids_.imu) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (frame_ids_.baselink == frame_ids_.camera) {
    return GetT_CAMERA_IMU(T);
  } else if (frame_ids_.baselink == frame_ids_.lidar) {
    return GetT_LIDAR_IMU(T);
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookupBase::GetT_IMU_BASELINK(Eigen::Matrix4d& T) {
  if (frame_ids_.baselink == frame_ids_.imu) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (frame_ids_.baselink == frame_ids_.camera) {
    return GetT_IMU_CAMERA(T);
  } else if (frame_ids_.baselink == frame_ids_.lidar) {
    return GetT_IMU_LIDAR(T);
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookupBase::GetT_BASELINK_CAMERA(Eigen::Matrix4d& T) {
  if (frame_ids_.baselink == frame_ids_.imu) {
    return GetT_IMU_CAMERA(T);
  } else if (frame_ids_.baselink == frame_ids_.camera) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (frame_ids_.baselink == frame_ids_.lidar) {
    return GetT_LIDAR_CAMERA(T);
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookupBase::GetT_CAMERA_BASELINK(Eigen::Matrix4d& T) {
  if (frame_ids_.baselink == frame_ids_.imu) {
    return GetT_CAMERA_IMU(T);
  } else if (frame_ids_.baselink == frame_ids_.camera) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (frame_ids_.baselink == frame_ids_.lidar) {
    return GetT_CAMERA_LIDAR(T);
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookupBase::GetT_BASELINK_LIDAR(Eigen::Matrix4d& T) {
  if (frame_ids_.baselink == frame_ids_.imu) {
    return GetT_IMU_LIDAR(T);
  } else if (frame_ids_.baselink == frame_ids_.camera) {
    return GetT_CAMERA_LIDAR(T);
  } else if (frame_ids_.baselink == frame_ids_.lidar) {
    T = Eigen::Matrix4d::Identity();
    return true;
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookupBase::GetT_LIDAR_BASELINK(Eigen::Matrix4d& T) {
  if (frame_ids_.baselink == frame_ids_.imu) {
    return GetT_LIDAR_IMU(T);
  } else if (frame_ids_.baselink == frame_ids_.camera) {
    return GetT_LIDAR_CAMERA(T);
  } else if (frame_ids_.baselink == frame_ids_.lidar) {
    T = Eigen::Matrix4d::Identity();
    return true;
  }

  // should not get here because of validation on init
  return false;
}

bool ExtrinsicsLookupBase::GetT_BASELINK_SENSOR(
    Eigen::Matrix4d& T, const std::string& sensor_frame) {
  if (!IsSensorFrameIdValid(sensor_frame)) {
    BEAM_ERROR("Input sensor frame id invalid.");
    return false;
  }

  if (sensor_frame == frame_ids_.baselink) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (sensor_frame == frame_ids_.imu) {
    return GetT_BASELINK_IMU(T);
  } else if (sensor_frame == frame_ids_.camera) {
    return GetT_BASELINK_CAMERA(T);
  } else if (sensor_frame == frame_ids_.lidar) {
    return GetT_BASELINK_LIDAR(T);
  } else {
    // shouldn't ever hit this due to sensor frame id check
    BEAM_ERROR("Input sensor frame id invalid.");
    return false;
  }
}

bool ExtrinsicsLookupBase::GetT_SENSOR_BASELINK(
    Eigen::Matrix4d& T, const std::string& sensor_frame) {
  if (GetT_BASELINK_SENSOR(T, sensor_frame)) {
    T = beam::InvertTransform(T);
    return true;
  } else {
    // error thrown by GetT_BASELINK_SENSOR
    return false;
  }
}

std::string ExtrinsicsLookupBase::GetImuFrameId() const {
  return frame_ids_.imu;
}

std::string ExtrinsicsLookupBase::GetCameraFrameId() const {
  return frame_ids_.camera;
}

std::string ExtrinsicsLookupBase::GetLidarFrameId() const {
  return frame_ids_.lidar;
}

std::string ExtrinsicsLookupBase::GetWorldFrameId() const {
  return frame_ids_.world;
}

std::string ExtrinsicsLookupBase::GetBaselinkFrameId() const {
  return frame_ids_.baselink;
}

bool ExtrinsicsLookupBase::IsSensorFrameIdValid(
    const std::string& sensor_frame) {
  if (sensor_frame == frame_ids_.imu || sensor_frame == frame_ids_.camera ||
      sensor_frame == frame_ids_.lidar) {
    return true;
  }
  return false;
}

bool ExtrinsicsLookupBase::GetTransform(Eigen::Matrix4d& T,
                                        const std::string& to_frame,
                                        const std::string& from_frame) {
  // TODO: this isn't the most efficient way to do this, might be better to do a
  // map, but for now this will do. We don't expect this function to be used
  // often anyways.
  if (to_frame == frame_ids_.baselink && from_frame == frame_ids_.imu) {
    return GetT_BASELINK_IMU(T);
  } else if (to_frame == frame_ids_.baselink &&
             from_frame == frame_ids_.camera) {
    return GetT_BASELINK_CAMERA(T);
  } else if (to_frame == frame_ids_.baselink &&
             from_frame == frame_ids_.lidar) {
    return GetT_BASELINK_LIDAR(T);
  } else if (to_frame == frame_ids_.baselink &&
             from_frame == frame_ids_.baselink) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (to_frame == frame_ids_.imu && from_frame == frame_ids_.baselink) {
    return GetT_IMU_BASELINK(T);
  } else if (to_frame == frame_ids_.imu && from_frame == frame_ids_.lidar) {
    return GetT_IMU_LIDAR(T);
  } else if (to_frame == frame_ids_.imu && from_frame == frame_ids_.camera) {
    return GetT_IMU_CAMERA(T);
  } else if (to_frame == frame_ids_.imu && from_frame == frame_ids_.imu) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (to_frame == frame_ids_.camera &&
             from_frame == frame_ids_.baselink) {
    return GetT_CAMERA_BASELINK(T);
  } else if (to_frame == frame_ids_.camera && from_frame == frame_ids_.imu) {
    return GetT_CAMERA_IMU(T);
  } else if (to_frame == frame_ids_.camera && from_frame == frame_ids_.lidar) {
    return GetT_CAMERA_LIDAR(T);
  } else if (to_frame == frame_ids_.camera && from_frame == frame_ids_.camera) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else if (to_frame == frame_ids_.lidar &&
             from_frame == frame_ids_.baselink) {
    return GetT_LIDAR_BASELINK(T);
  } else if (to_frame == frame_ids_.lidar && from_frame == frame_ids_.imu) {
    return GetT_LIDAR_IMU(T);
  } else if (to_frame == frame_ids_.lidar && from_frame == frame_ids_.camera) {
    return GetT_LIDAR_CAMERA(T);
  } else if (to_frame == frame_ids_.lidar && from_frame == frame_ids_.lidar) {
    T = Eigen::Matrix4d::Identity();
    return true;
  } else {
    BEAM_ERROR("Invalid to_frame and from_frame combination.");
    return false;
  }
}

bool ExtrinsicsLookupBase::SetTransform(const Eigen::Matrix4d& T,
                                        const std::string& to_frame,
                                        const std::string& from_frame) {
  if (!IsSensorFrameIdValid(to_frame) || !IsSensorFrameIdValid(from_frame) ||
      to_frame == from_frame) {
    BEAM_ERROR(
        "Cannot set transform, to_frame and/or from_frame invalid. to_frame: "
        "{}, "
        "from_frame: {}",
        to_frame, from_frame);
    return false;
  }

  // TODO: this isn't the most efficient way to do this, might be better to do a
  // map, but for now this will do. We don't expect this function to be used
  // often anyways.
  if (to_frame == frame_ids_.imu && from_frame == frame_ids_.lidar) {
    T_LIDAR_IMU_ = beam::InvertTransform(T);
    T_LIDAR_IMU_set_ = true;
  } else if (to_frame == frame_ids_.imu && from_frame == frame_ids_.camera) {
    T_IMU_CAMERA_ = T;
    T_IMU_CAMERA_set_ = true;
  } else if (to_frame == frame_ids_.camera && from_frame == frame_ids_.imu) {
    T_IMU_CAMERA_ = beam::InvertTransform(T);
    T_IMU_CAMERA_set_ = true;
  } else if (to_frame == frame_ids_.camera && from_frame == frame_ids_.lidar) {
    T_LIDAR_CAMERA_ = beam::InvertTransform(T);
    T_LIDAR_CAMERA_set_ = true;
  } else if (to_frame == frame_ids_.lidar && from_frame == frame_ids_.imu) {
    T_LIDAR_IMU_ = T;
    T_LIDAR_IMU_set_ = true;
  } else if (to_frame == frame_ids_.lidar && from_frame == frame_ids_.camera) {
    T_LIDAR_CAMERA_ = T;
    T_LIDAR_CAMERA_set_ = true;
  } else {
    BEAM_ERROR("Invalid to_frame and from_frame combination.");
    return false;
  }
  return true;
}

}  // namespace bs_common