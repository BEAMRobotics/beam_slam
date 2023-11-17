#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_cv/OpenCVConversions.h>
#include <beam_utils/pointclouds.h>
#include <bs_models/frame_initializers/frame_initializer.h>

struct ScanStamped {
  pcl::PointCloud<PointXYZIRT> scan;
  ros::Time start;
  ros::Time end;
  ros::Time stamp;
};

struct ImageStamped {
  cv::Mat image;
  ros::Time stamp;
};

struct CalibrationViewer {
  CalibrationViewer() : nh_("~") {
    LoadParams();
    Setup();
  }

  void LoadParams() {
    if (!nh_.getParam("lidar_topic", lidar_topic_)) {
      ROS_ERROR("Failed to load parameter 'lidar_topic'");
      throw std::runtime_error("Failed to load parameter 'lidar_topic'");
    }
    if (!nh_.getParam("image_topic", image_topic_)) {
      ROS_ERROR("Failed to load parameter 'image_topic'");
      throw std::runtime_error("Failed to load parameter 'image_topic'");
    }

    std::string frame_initializer_config_rel;
    if (!nh_.getParam("frame_initializer_config",
                      frame_initializer_config_rel)) {
      ROS_ERROR("Failed to load parameter 'frame_initializer_config'");
      throw std::runtime_error(
          "Failed to load parameter 'frame_initializer_config'");
    }
    frame_init_config_ = beam::CombinePaths(bs_common::GetBeamSlamConfigPath(),
                                            frame_initializer_config_rel);

    std::string camera_intrinsics_path_rel;
    const std::string intrinsics_topic =
        "/calibration_params/camera_intrinsics_path";
    if (!nh_.getParam(intrinsics_topic, camera_intrinsics_path_rel)) {
      ROS_ERROR("Failed to load parameter {}", intrinsics_topic);
      throw std::runtime_error("Failed to load intrinsics parameter");
    }
    camera_intrinsics_path_ = beam::CombinePaths(
        bs_common::GetBeamSlamCalibrationsPath(), camera_intrinsics_path_rel);
  }

  void Setup() {
    image_subscriber_ =
        nh_.subscribe(image_topic, 10, &CalibrationViewer::ImageCallback, this);
    lidar_subscriber_ = nh_.subscribe(lidar_topic_, 10,
                                      &CalibrationViewer::LidarCallback, this);
    image_publisher_ =
        nh_.advertise<sensor_msgs::Image>("/calibration_viewer/image", 10);

    frame_initializer_ =
        std::make_unique<bs_models::FrameInitializer>(frame_init_config_);

    camera_model_ =
        beam_calibration::CameraModel::Create(camera_intrinsics_path_);
  }

  void ImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ImageStamped i;
    i.image = beam_cv::OpenCVConversions::RosImgToMat(*msg);
    i.stamp = msg->header.stamp;
    ProcessData();
  }

  void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ScanStamped s;
    beam::ROSToPCL(s.scan, *msg);
    s.stamp = msg->header.stamp;
    s.end = ros::Time(0);
    s.start = ros::TIME_MAX;
    for (const auto& p : s.scan) {
      ros::Time pt = cloud_stamp + ros::Duration(p.time);
      if (pt < s.start) { s.start = pt; }
      if (pt > s.end) { s.end = pt; }
    }
    scans_.emplace(s);
  }

  void ProcessData() {
    if (scans_.empty() || images_.empty()) { return; }

    while (!images_.empty()) {
      const auto& image = images.front();
      while (!scans_.empty()) {
        const auto& scan = scans_.front();
        if (scan.end < image.stamp) {
          // remove lidar data before the image
          scans_.pop();
          continue;
        } else if (scan.start > image.stamp) {
          // remove image which is before the first lidar scan
          images_.pop();
          continue;
        } else if (scan.start <= image.stamp && scan.end >= image.stamp) {
          pcl::PointCloud<PointXYZIRT> cloud_aligned =
              AlignScanToTime(scan.scan, image.stamp);
          DisplayData(cloud_aligned, image.image, image.stamp);
          images_.pop();
        }
      }
    }

    // if image buffer gets too large, start deleting
    while (images_.size() > 3) { images_.pop(); }
  }

  pcl::PointCloud<PointXYZIRT>
      AlignScanToTime(const pcl::PointCloud<PointXYZIRT>& scan,
                      const ros::Time& stamp) {
    pcl::PointCloud<PointXYZIRT> cloud_aligned;
    for (const auto& p : scan) {
      ros::Time pt = stamp + ros::Duration(p.time);
      Eigen::Matrix4d T_World_LidarN;
      if (!frame_initializer_->GetPose(T_World_LidarN, pt,
                                       extrinsics_.GetLidarFrameId())) {
        // if any point doesn't return a valid result, then skip
        break;
      }
      Eigen::Affine3f T_Lidar0_LidarN(T_Lidar0_World *
                                      T_World_LidarN.cast<float>());
      PointXYZIRT p_aligned =
          pcl::transformPoint<PointXYZIRT>(p, T_Lidar0_LidarN);
      cloud_aligned.push_back(p_aligned);
    }
    return cloud_aligned;
  }

  void DisplayData(const pcl::PointCloud<PointXYZIRT>& cloud, cv::Mat& image,
                   const ros::Time& stamp) {
    Eigen::Matrix4d T_Cam_Lidar;
    if (!extrinsics_.GetT_CAMERA_LIDAR(T_Cam_Lidar, stamp)) {
      ROS_WARN("cannot get extrinsics, skipping image");
      return;
    }
    pcl::PointCloud<PointXYZIRT> cloud_in_cam;
    pcl::transformPointCloud(cloud, cloud_in_cam, Eigen::Affine3d(T_Lidar_Cam));

    std::vector<Eigen::Vector2d> pixels;
    cv::Mat depth_image(image.rows, image.cols, cv::CV_32F);
    for (const auto& p : cloud_in_cam) {
      if (p.z < 0) { continue; }
      Eigen::Vector2d pixel;
      Eigen::Vector3d p_in_cam(p.x, p.y, p.z);
      bool in_image;
      if (!camera_model->ProjectPoint(p_in_cam, pixel, in_image)) { continue; }

      if (!in_image) { continue; }
      pixels.push_back(pixel);
      int row = static_cast<int>(pixel[1]);
      int col = static_cast<int>(pixel[0]);
      depth_image.at(row, col) = p_in_cam.norm();
    }

    cv::Mat depth_image_col;
    cv::applyColorMap(depth_image, depth_image_col, cv::COLORMAP_JET);

    static int radius = 2;
    for (const auto& pixel : pixels) {
      int row = static_cast<int>(pixels[1]);
      int col = static_cast<int>(pixels[0]);
      cv::Point p(pixel[0], pixel[1]);
      cv::circle(image, p, radius, depth_image_col.at(row, col));
    }

    std_msgs::Header header;
    header.stamp = stamp;
    header.seq = publisher_counter_++;
    header.frame_id = extrinsics_->GetCameraFrameId();
    sensor_msgs::Image out_msg =
        beam_cv::OpenCVConversions::MatToRosImg(image, header, "bgr8");
    image_publisher_.publish(out_msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber image_subscriber_;
  ros::Subscriber lidar_subscriber_;
  ros::Publisher image_publisher_;
  std::string lidar_topic_;
  std::string image_topic_;
  std::string frame_init_config_;
  std::string camera_intrinsics_path_;
  std::unique_ptr<bs_models::FrameInitializer> frame_initializer_;
  bs_common::ExtrinsicsLookupOnline& extrinsics_ =
      bs_common::ExtrinsicsLookupOnline::GetInstance();
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;

  int publisher_counter_{0};
  std::queue<ScanStamped> scans_;
  std::queue<ImageStamped> images_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "calibration_viewer");
  CalibrationViewer viewer;
  ros::spin();

  return 0;
}
