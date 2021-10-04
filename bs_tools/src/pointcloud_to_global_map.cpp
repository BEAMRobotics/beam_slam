#include <bs_tools/pointcloud_to_global_map.h>

#include <algorithm>

#include <beam_calibration/Radtan.h>
#include <beam_filtering/CropBox.h>
#include <beam_utils/log.h>

#include <bs_common/extrinsics_lookup_base.h>
#include <bs_common/utils.h>
#include <bs_models/global_mapping/submap.h>

namespace bs_tools {

PointcloudToGlobalMap::Params::Params() {
  // no complex parameters rn
}

void PointcloudToGlobalMap::Params::LoadJson(const std::string &config_path) {
  std::string read_file = config_path;
  if (read_file.empty()) {
    BEAM_INFO("No config file provided to pointcloud to global map converter, "
              "using default parameters.");
    return;
  }

  if (read_file == "DEFAULT_PATH") {
    read_file =
        bs_common::GetBeamSlamConfigPath() + "global_map/global_map.json";
  }

  BEAM_INFO("Loading global map config file: {}", read_file);

  nlohmann::json J;
  if (!beam::ReadJson(read_file, J)) {
    BEAM_ERROR("Using default parameters.");
    return;
  }

  submap_distance = J["submap_distance_m"];
  submap_size = J["submap_size_m"];

  if (submap_distance > submap_size) {
    BEAM_WARN("Submap size less than submap distance specified.  Submaps will "
              "not overlap, and some pointcloud regions will be lost.");
  }

  nominal_gravity_direction = J["nominal_gravity_direction"];
  if (nominal_gravity_direction != "-X" && nominal_gravity_direction != "+X" &&
      nominal_gravity_direction != "-Y" && nominal_gravity_direction != "+Y" &&
      nominal_gravity_direction != "-Z" && nominal_gravity_direction != "+Z") {
    BEAM_ERROR(
        "nominal_gravity_direction must be + or - followed by X, Y, or Z");
  }
}

PointcloudToGlobalMap::PointcloudToGlobalMap(const std::string &pointcloud,
                                             const Params &params)
    : pointcloud_(pointcloud), params_(params) {
  CreateGlobalMap();
}

PointcloudToGlobalMap::PointcloudToGlobalMap(const std::string &pointcloud,
                                             const std::string &config_path)
    : pointcloud_(pointcloud) {
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
      "base_link", "camera_link", "lidar_link", "base_link", "base_link"};
  std::shared_ptr<bs_common::ExtrinsicsLookupBase> extrinsics =
      std::make_shared<bs_common::ExtrinsicsLookupBase>(frame_ids);
  extrinsics->SetTransform(Eigen::Matrix4d::Identity(), "lidar_link",
                           "base_link");
  global_map_ = std::make_shared<bs_models::global_mapping::GlobalMap>(
      camera, extrinsics);
  std::vector<std::shared_ptr<bs_models::global_mapping::Submap>> submaps;

  // construct clouds and cropbox
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  beam_filtering::CropBox<pcl::PointXYZ> cropbox;
  cropbox.SetRemoveOutsidePoints(true);

  // read point cloud and add to cropbox
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_, *cloud) ==
      -1) //* load the file
  {
    BEAM_ERROR("Couldn't read valid PCD file at location {}", pointcloud_);
    return false;
  }
  BEAM_INFO("Loaded pointcloud from PCD file at location {}", pointcloud_);
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
  }

  // distribute extra map on either side of submap grid
  int l_n = (int)(l / params_.submap_distance);
  i += (l - (l_n * params_.submap_distance)) / 2;
  int w_n = (int)(w / params_.submap_distance);
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
  // verify output_path
  if (!boost::filesystem::exists(output_path)) {
    BEAM_INFO("Output directory does not exist, can not save global map "
              "data. Input: {}",
              output_path);
    return false;
  }

  // save
  global_map_->SaveData(output_path);
  return true;
}

bool PointcloudToGlobalMap::SaveGlobalMapResults(
    const std::string &output_path) {
  // verify output_path
  if (!boost::filesystem::exists(output_path)) {
    BEAM_INFO("Output directory does not exist, can not save global map "
              "results. Input: {}",
              output_path);
    return false;
  }

  // save
  global_map_->SaveTrajectoryFile(output_path);
  global_map_->SaveTrajectoryClouds(output_path);
  global_map_->SaveSubmapFrames(output_path);
  global_map_->SaveLidarSubmaps(output_path);
  global_map_->SaveKeypointSubmaps(output_path);
  return true;
}

} // namespace bs_tools