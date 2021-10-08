#include <bs_tools/pointcloud_to_global_map.h>

#include <algorithm>

#include <beam_calibration/Radtan.h>
#include <beam_filtering/CropBox.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>
#include <beam_utils/pointclouds.h>

#include <bs_common/extrinsics_lookup_base.h>
#include <bs_common/utils.h>
#include <bs_models/global_mapping/submap.h>

namespace bs_tools {

void PointcloudToGlobalMap::Params::LoadJson(const std::string &config_path) {
  std::string read_file = config_path;
  if (read_file.empty()) {
    BEAM_INFO("No config file provided to pointcloud to global map converter, "
              "using default parameters.");
    return;
  }

  if (read_file == "DEFAULT_PATH") {
    read_file = bs_common::GetBeamSlamConfigPath() +
                "global_map/pointcloud_to_global_map.json";
  }

  BEAM_INFO("Loading global map config file: {}", read_file);

  nlohmann::json J;
  if (!beam::ReadJson(read_file, J)) {
    BEAM_ERROR("Using default parameters.");
    return;
  }

  try {
    submap_distance = J["submap_distance_m"];
  } catch (const std::exception &e) {
    BEAM_ERROR("Could not read submap_distance_m from JSON.");
  }
  try {
    submap_size = J["submap_size_m"];
  } catch (const std::exception &e) {
    BEAM_ERROR("Could not read submap_size_m from JSON.");
  }
  if (submap_distance > submap_size) {
    BEAM_WARN("Submap size less than submap distance specified.  Submaps will "
              "not overlap, and some pointcloud regions will be lost.");
  }

  try {
    nominal_gravity_direction = J["nominal_gravity_direction"];
  } catch (const std::exception &e) {
    BEAM_ERROR("Could not read nominal_gravity_direction from JSON.");
  }
  if (nominal_gravity_direction != "-X" && nominal_gravity_direction != "+X" &&
      nominal_gravity_direction != "-Y" && nominal_gravity_direction != "+Y" &&
      nominal_gravity_direction != "-Z" && nominal_gravity_direction != "+Z") {
    BEAM_ERROR(
        "nominal_gravity_direction must be + or - followed by X, Y, or Z");
  }
}

PointcloudToGlobalMap::PointcloudToGlobalMap(const std::string &pointcloud_path,
                                             const Params &params)
    : pointcloud_(pointcloud_path), params_(params) {
  CreateGlobalMap();
}

PointcloudToGlobalMap::PointcloudToGlobalMap(const std::string &pointcloud_path,
                                             const std::string &config_path)
    : pointcloud_(pointcloud_path) {
  params_.LoadJson(config_path);
  CreateGlobalMap();
}

bool PointcloudToGlobalMap::CreateGlobalMap() {
  // construct global map and submaps
  std::shared_ptr<beam_calibration::CameraModel> camera =
      std::make_shared<beam_calibration::Radtan>(
          0, 0, Eigen::Matrix<double, 8, 1>::Zero());
  // simulate lidar extrinsics with all identity transformations
  bs_common::ExtrinsicsLookupBase::FrameIds frame_ids{
      "imu_link", "camera_link", "lidar_link", "world", "imu_link"};
  std::shared_ptr<bs_common::ExtrinsicsLookupBase> extrinsics =
      std::make_shared<bs_common::ExtrinsicsLookupBase>(frame_ids);
  Eigen::Matrix4d eye = Eigen::Matrix4d::Identity();
  extrinsics->SetTransform(eye, frame_ids.imu, frame_ids.camera);
  extrinsics->SetTransform(eye, frame_ids.imu, frame_ids.lidar);
  global_map_ = std::make_shared<bs_models::global_mapping::GlobalMap>(
      camera, extrinsics);
  std::vector<std::shared_ptr<bs_models::global_mapping::Submap>> submaps;

  // construct clouds and cropbox
  PointCloudPtr cloud = std::make_shared<PointCloud>();
  PointCloudPtr subcloud = std::make_shared<PointCloud>();
  beam_filtering::CropBox<pcl::PointXYZ> cropbox;
  cropbox.SetRemoveOutsidePoints(true);

  // read point cloud and add to cropbox
  int read_status;
  if (beam::GetExtension(pointcloud_) == ".pcd") {
    read_status = pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_, *cloud);
  } else if (beam::GetExtension(pointcloud_) == ".ply") {
    read_status = pcl::io::loadPLYFile<pcl::PointXYZ>(pointcloud_, *cloud);
  }
  if (read_status == -1) {
    BEAM_ERROR("Couldn't read valid PCD or PLY file at location {}",
               pointcloud_);
    return false;
  }
  BEAM_INFO("Loaded pointcloud from file at location {}", pointcloud_);
  cropbox.SetInputCloud(cloud);

  // establish a system to iterate through center points
  // using l and w, since gravity could be x, y or z
  // set cropbox size (centered at origin to be transformed around)
  double l;     // bounding length
  double w;     // bounding width
  double l_max; // bounding edge in pc frame
  double w_max; // bounding edge in pc frame
  double i;     // current center point along length
  double j;     // current center point along width
  Eigen::Vector3f i_vec;
  Eigen::Vector3f j_vec;
  Eigen::Matrix3f R_box_cloud;

  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D(*cloud, min, max);
  bool dimension_check;
  if (params_.nominal_gravity_direction.back() == 'Z') {
    // length [i] direction => X
    // width [j] direction => Y
    l = max.x - min.x;
    w = max.y - min.y;
    l_max = max.x;
    w_max = max.y;
    i = min.x;
    j = min.y;
    i_vec = Eigen::Vector3f::UnitX();
    j_vec = Eigen::Vector3f::UnitY();
    R_box_cloud = Eigen::Matrix3f::Identity();
    dimension_check = (max.z - min.z) < l_max && (max.z - min.z) < w_max;
  } else if (params_.nominal_gravity_direction.back() == 'Y') {
    // length [i] direction => X
    // width [j] direction => Z
    l = max.x - min.x;
    w = max.z - min.z;
    l_max = max.x;
    w_max = max.z;
    i = min.x;
    j = min.z;
    i_vec = Eigen::Vector3f::UnitX();
    j_vec = Eigen::Vector3f::UnitZ();
    R_box_cloud = Eigen::AngleAxisf(M_PI / -2, Eigen::Vector3f::UnitX());
    dimension_check = (max.y - min.y) < l_max && (max.y - min.y) < w_max;
  } else if (params_.nominal_gravity_direction.back() == 'X') {
    // length [i] direction => Y
    // width [j] direction => Z
    l = max.y - min.y;
    w = max.z - min.z;
    l_max = max.y;
    w_max = max.z;
    i = min.y;
    j = min.z;
    i_vec = Eigen::Vector3f::UnitY();
    j_vec = Eigen::Vector3f::UnitZ();
    R_box_cloud = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY());
    dimension_check = (max.x - min.x) < l_max && (max.x - min.x) < w_max;
  }
  if (!dimension_check) {
    BEAM_INFO("The current nominal gravity direction is not the smallest "
              "dimension. Proceeding with warning.");
  }

  // distribute extra map on either side of submap grid
  int l_n = static_cast<int>(l / params_.submap_distance);
  i += (l - (l_n * params_.submap_distance)) / 2;
  int w_n = static_cast<int>(w / params_.submap_distance);
  j += (w - (w_n * params_.submap_distance)) / 2;

  // configure cropbox to handle transforms for any nominal gravity
  double absolute_min = std::min(std::min(min.x, min.y), min.z);
  double absolute_max = std::max(std::max(max.x, max.y), max.z);
  Eigen::Vector3f crop_min(params_.submap_size / -2, params_.submap_size / -2,
                           absolute_min);
  Eigen::Vector3f crop_max(params_.submap_size / 2, params_.submap_size / 2,
                           absolute_max);
  cropbox.SetMinVector(crop_min);
  cropbox.SetMaxVector(crop_max);

  // iteratively crop submaps
  Eigen::Matrix4f T_box_cloud = Eigen::Matrix4f::Identity();
  T_box_cloud.block(0, 0, 3, 3) = R_box_cloud;
  while (i < l_max) {
    while (j < w_max) {
      Eigen::Vector3f t_box_cloud = -i * i_vec - j * j_vec;
      T_box_cloud.block(0, 3, 3, 1) = t_box_cloud;
      cropbox.SetTransform(T_box_cloud);
      cropbox.Filter();
      *subcloud = cropbox.GetFilteredCloud();

      if (subcloud->points.size() > 0) {
        BEAM_INFO("Adding submap of size {}", subcloud->points.size());
        ros::Time::init();
        ros::Time time = ros::Time::now();
        std::shared_ptr<bs_models::global_mapping::Submap> submap =
            std::make_shared<bs_models::global_mapping::Submap>(
                time, beam::InvertTransform(T_box_cloud.cast<double>()), camera,
                extrinsics);
        submap->AddLidarMeasurement(*subcloud, Eigen::Matrix4d::Identity(),
                                    time, 0);
        submaps.push_back(submap);
      }

      j += params_.submap_distance; // iterate j
    }
    j -= (w_n + 1) * params_.submap_distance; // reset j
    i += params_.submap_distance;             // iterate i
  }

  global_map_->SetOnlineSubmaps(submaps);

} // namespace bs_tools

bool PointcloudToGlobalMap::SaveGlobalMapData(const std::string &output_path) {
  // save
  global_map_->SaveData(output_path);
  return true;
}

bool PointcloudToGlobalMap::SaveGlobalMapResults(
    const std::string &output_path) {
  // save
  global_map_->SaveSubmapFrames(output_path, false);
  global_map_->SaveLidarSubmaps(output_path, false);
  return true;
}

} // namespace bs_tools