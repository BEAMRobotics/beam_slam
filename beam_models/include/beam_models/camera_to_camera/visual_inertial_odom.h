#pragma once
// std
#include <queue>

// messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <beam_models/InitializedPathMsg.h>

// fuse
#include <fuse_core/async_sensor_model.h>

// beam_slam
#include <beam_models/camera_to_camera/visual_map.h>
#include <beam_models/frame_to_frame/imu_preintegration.h>
#include <beam_models/trajectory_initializers/vio_initializer.h>
#include <beam_parameters/models/camera_params.h>
#include <beam_parameters/models/global_params.h>

// libbeam
#include <beam_calibration/CameraModel.h>
#include <beam_cv/geometry/PoseRefinement.h>
#include <beam_cv/trackers/Trackers.h>

namespace beam_models { namespace camera_to_camera {

class VisualInertialOdom : public fuse_core::AsyncSensorModel {
public:
  SMART_PTR_DEFINITIONS(VisualInertialOdom);

  /**
   * @brief Default Constructor
   */
  VisualInertialOdom();

  /**
   * @brief Default Destructor
   */
  ~VisualInertialOdom() override = default;

  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processImage(const sensor_msgs::Image::ConstPtr& msg);

  /**
   * @brief Callback for image processing, this callback will add visual
   * constraints and triangulate new landmarks when required
   * @param[in] msg - The image to process
   */
  void processIMU(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback for path processing, this path is provided by LIO for
   * initialization
   * @param[in] msg - The path to process
   */
  void processInitPath(const InitializedPathMsg::ConstPtr& msg);

protected:
  fuse_core::UUID device_id_; //!< The UUID of this device

  /**
   * @brief Perform any required initialization for the sensor model
   *
   * This could include things like reading from the parameter server or
   * subscribing to topics. The class's node handles will be properly
   * initialized before SensorModel::onInit() is called. Spinning of the
   * callback queue will not begin until after the call to SensorModel::onInit()
   * completes.
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the
   * optimizer
   */
  void onStart() override {}

  /**
   * @brief Unsubscribe to the input topic
   */
  void onStop() override;

  /**
   * @brief Callback for when a newly optimized graph is available
   */
  void onGraphUpdate(fuse_core::Graph::ConstSharedPtr graph_msg) override;

private:
  /**
   * @brief Converts ros image message to opencv image
   * @param msg image message to convert to cv mat
   */
  cv::Mat ExtractImage(const sensor_msgs::Image& msg);

  /**
   * @brief Copies all variables and constraints in the init graph and sends to
   * fuse
   * @param init_graph the graph obtained from the initializer
   */
  void SendInitializationGraph(const fuse_graphs::HashGraph& init_graph);

  /**
   * @brief Localizes a given frame using the tracker and the current visual map
   * @param img_time time of image to localize
   * @param[out] triangulated_ids id's of landmarks that have already been
   * triangulated
   * @param[out] untriangulated_ids id's of landmarks that have not been
   * triangulated
   * @return T_WORLD_CAMERA
   */
  Eigen::Matrix4d LocalizeFrame(const ros::Time& img_time,
                                std::vector<uint64_t>& triangulated_ids,
                                std::vector<uint64_t>& untriangulated_ids);

  /**
   * @brief Determines if a frame is a keyframe
   * @param img_time time of image to determine if its a keyframe
   * @param triangulated_ids id's of landmarks that have already been
   * triangulated
   * @param untriangulated_ids id's of landmarks that have not been
   * triangulated
   * @return true or false decision
   */
  bool IsKeyframe(const ros::Time& img_time,
                  const std::vector<uint64_t>& triangulated_ids,
                  const std::vector<uint64_t>& untriangulated_ids);

protected:
  // loadable camera parameters
  beam_parameters::models::CameraParams camera_params_;
  
  // global parameters
  beam_parameters::models::GlobalParams global_params_;

  // topic subscribers and buffers
  ros::Subscriber image_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber path_subscriber_;
  std::queue<sensor_msgs::Image> image_buffer_;
  std::queue<sensor_msgs::Imu> imu_buffer_;

  // computer vision objects
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  std::shared_ptr<beam_models::camera_to_camera::VisualMap> visual_map_;

  // initialization object
  std::shared_ptr<beam_models::camera_to_camera::VIOInitializer> initializer_;

  // imu preintegration object
  std::shared_ptr<beam_models::frame_to_frame::ImuPreintegration> imu_preint_;

  // most recent keyframe timestamp
  ros::Time cur_kf_time_ = ros::Time(0);
  std::deque<ros::Time> keyframes_;

  // temp stuff
  ros::Time last_stamp_;
  InitializedPathMsg init_path_;
  bool set_once = false;
  ros::Publisher init_path_pub_;
  void BuildPath() {
    nav_msgs::Path path;
    Eigen::Matrix4d T_body_vicon_;
    T_body_vicon_ << 0.33638, -0.01749, 0.94156, 0.06901, //
        -0.02078, -0.99972, -0.01114, -0.02781,           //
        0.94150, -0.01582, -0.33665, -0.12395,            //
        0.0, 0.0, 0.0, 1.0;                               //
    // read path
    std::string file = "/home/jake/data/vicon_path.txt";
    std::ifstream infile;
    std::string line;
    // open file
    infile.open(file);
    while (!infile.eof()) {
      ros::Time stamp;
      // get timestamp k
      std::getline(infile, line, ',');
      uint64_t t = std::stod(line);
      stamp.fromNSec(t);
      std::getline(infile, line, ',');
      double xp = std::stod(line);
      std::getline(infile, line, ',');
      double yp = std::stod(line);
      std::getline(infile, line, ',');
      double zp = std::stod(line);
      std::getline(infile, line, ',');
      double xr = std::stod(line);
      std::getline(infile, line, ',');
      double yr = std::stod(line);
      std::getline(infile, line, ',');
      double zr = std::stod(line);
      std::getline(infile, line, '\n');
      double wr = std::stod(line);

      Eigen::Vector3d position(xp, yp, zp);
      Eigen::Quaterniond orientation;
      orientation.w() = wr;
      orientation.x() = xr;
      orientation.y() = yr;
      orientation.z() = zr;
      Eigen::Matrix4d T_world_vicon;
      beam::QuaternionAndTranslationToTransformMatrix(orientation, position,
                                                      T_world_vicon);
      Eigen::Matrix4d T_world_imu = T_body_vicon_ * T_world_vicon;

      Eigen::Vector3d position_p;
      Eigen::Quaterniond orientation_p;
      beam::TransformMatrixToQuaternionAndTranslation(
          T_world_imu, orientation_p, position_p);

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.pose.position.x = position_p[0];
      pose.pose.position.y = position_p[1];
      pose.pose.position.z = position_p[2];
      pose.pose.orientation.x = orientation_p.x();
      pose.pose.orientation.y = orientation_p.y();
      pose.pose.orientation.z = orientation_p.z();
      pose.pose.orientation.w = orientation_p.w();
      path.poses.push_back(pose);
      last_stamp_ = stamp;
    }
    init_path_.poses = path.poses;
    init_path_.scale = 0;
    init_path_.gyroscope_bias.x = 0;
    init_path_.gyroscope_bias.y = 0;
    init_path_.gyroscope_bias.z = 0;
    init_path_.accelerometer_bias.x = 0;
    init_path_.accelerometer_bias.y = 0;
    init_path_.accelerometer_bias.z = 0;
    init_path_.gravity.x = 0;
    init_path_.gravity.y = 0;
    init_path_.gravity.z = 0;
  }
};

}} // namespace beam_models::camera_to_camera
