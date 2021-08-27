#pragma once

#include <Eigen/Dense>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>

namespace bs_common {

/**
 * @brief this class is used to keep extrinsics calibration for one time
 * instance. The 3 types of frames need to be supplied to this class.
 */
class ExtrinsicsLookupBase {
 public:
  /**
   * @brief struct for containing all required frame id inputs to this class
   */
  struct FrameIds {
    std::string imu{""};
    std::string camera{""};
    std::string lidar{""};
    std::string world{""};
    std::string baselink{""};
  };

  /**
   * @brief delete default constructor
   */
  ExtrinsicsLookupBase() = delete;

  /**
   * @brief constructor requiring a frame ids struct
   * @param frame_ids see above
   */
  ExtrinsicsLookupBase(const FrameIds& frame_ids);

  /**
   * @brief constructor requiring frame ids and a path to a json with the
   * extrinsics.
   * @param frame_ids see struct above
   * @param extrinsics_filepath full path to extrinsics json file. For format,
   * see SaveToJson function
   */
  ExtrinsicsLookupBase(const FrameIds& frame_ids,
                       const std::string& extrinsics_filepath);

  /**
   * @brief constructor requiring a path to a json with the file ids, and a path
   * to a json with the extrinsics.
   * @param frame_ids full path to frame ids json file. For format,
   * see SaveToJson function
   * @param extrinsics_filepath full path to extrinsics json file. For format,
   * see SaveToJson function
   */
  ExtrinsicsLookupBase(const std::string& frame_ids_filepath,
                       const std::string& extrinsics_filepath);

  /**
   * @brief Default copy constructor
   */
  ExtrinsicsLookupBase(const ExtrinsicsLookupBase& other) = default;

  /**
   * @brief Default copy assignment operator
   */
  ExtrinsicsLookupBase& operator=(const ExtrinsicsLookupBase& other) = default;

  /**
   * @brief check that frame ids are valid, if not throw exception
   */
  void ValidateFrameIds();

  /**
   * @brief Load extrinscs from json. See SaveExtrinsicsToJson for format
   * @param filepath full path to json file
   */
  void LoadExtrinsics(const std::string& filepath);

  /**
   * @brief Save all extrinsics to a json file;. For output format, see
   * extrinsics.json examples in beam_calibration/tests
   * @param save_filename full path to filename to save. E.g.,
   * /tmp/extrinsics/extrinsics.json. Directory to save file must exist.
   */
  void SaveExtrinsicsToJson(const std::string& save_filename);

  /**
   * @brief Save all frame ids to a json file.
   * @param save_filename full path to filename to save. E.g.,
   * /tmp/extrinsics/frame_ids.json. Directory to save file must exist.
   */
  void SaveFrameIdsToJson(const std::string& save_filename);

  /**
   * @brief Gets the extrinsics between camera and IMU
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_CAMERA_IMU(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between IMU and camera
   * @param T reference to result
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_IMU_CAMERA(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between camera and lidar
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_CAMERA_LIDAR(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between lidar and camera
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_LIDAR_CAMERA(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between IMU and lidar
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_IMU_LIDAR(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between lidar and IMU
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_LIDAR_IMU(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between baselink and IMU
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_BASELINK_IMU(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between baselink and camera
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_BASELINK_CAMERA(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between baselink and lidar
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_BASELINK_LIDAR(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between baselink and sensor
   * @param T reference to result
   * @param sensor_frame sensor frame id
   * @return true if lookup was successful
   */
  bool GetT_BASELINK_SENSOR(Eigen::Matrix4d& T,
                            const std::string& sensor_frame);

  /**
   * @brief Gets the extrinsics between IMU and baselink
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_IMU_BASELINK(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between camera and baselink
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_CAMERA_BASELINK(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between lidar and baselink
   * @param T reference to result
   * @return true if lookup was successful
   */
  bool GetT_LIDAR_BASELINK(Eigen::Matrix4d& T);

  /**
   * @brief Gets the extrinsics between sensor and baselink
   * @param T reference to result
   * @param sensor_frame sensor frame id
   * @param time extrinsics time if extrinsics are not static
   * @return true if lookup was successful
   */
  bool GetT_SENSOR_BASELINK(Eigen::Matrix4d& T,
                            const std::string& sensor_frame);

  /**
   * @brief Gets the frame id of IMU
   * @return frame id
   */
  std::string GetImuFrameId() const;

  /**
   * @brief Gets the frame id of camera
   * @return frame id
   */
  std::string GetCameraFrameId() const;

  /**
   * @brief Gets the frame id of lidar
   * @return frame id
   */
  std::string GetLidarFrameId() const;

  /**
   * @brief Gets the frame id of the world frame
   * @return frame id
   */
  std::string GetWorldFrameId() const;

  /**
   * @brief Gets the frame id of the baselink frame
   * @return frame id
   */
  std::string GetBaselinkFrameId() const;

  /**
   * @brief Verifies if sensor frame id is valid by checking against IMU,
   * camera, or lidar frame id
   * @return true if sensor frame id matches any of these frames
   */
  bool IsSensorFrameIdValid(const std::string& sensor_frame);

  /**
   * @brief Gets transform between specified frames
   * @param T reference to result
   * @param to_frame 'to frame' of transformation
   * @param from_frame 'from frame' of transformation
   * @return true if lookup was successful
   */
  bool GetTransform(Eigen::Matrix4d& T, const std::string& to_frame,
                    const std::string& from_frame);

  /**
   * @brief Set some transform between specified frames
   * @param T transform to set
   * @param to_frame 'to frame' of transformation
   * @param from_frame 'from frame' of transformation
   * @return true successful
   */
  bool SetTransform(const Eigen::Matrix4d& T, const std::string& to_frame,
                    const std::string& from_frame);

 private:
  Eigen::Matrix4d T_LIDAR_IMU_;
  Eigen::Matrix4d T_LIDAR_CAMERA_;
  Eigen::Matrix4d T_IMU_CAMERA_;

  FrameIds frame_ids_;

  bool T_LIDAR_IMU_set_{false};
  bool T_LIDAR_CAMERA_set_{false};
  bool T_IMU_CAMERA_set_{false};
};

}  // namespace bs_common
