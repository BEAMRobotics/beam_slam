#include <bs_models/vision/utils.h>

#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/geometry/Triangulation.h>
#include <beam_utils/utils.h>
#include <bs_models/vision/visual_map.h>
#include <fuse_graphs/hash_graph.h>
#include <fuse_loss/cauchy_loss.h>

#include <algorithm>

namespace bs_models { namespace vision {

std::map<uint64_t, Eigen::Matrix4d> ComputePathWithVision(
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const std::shared_ptr<beam_containers::LandmarkContainer>&
        landmark_container,
    const Eigen::Matrix4d& T_camera_baselink,
    fuse_core::Loss::SharedPtr loss_function, double max_optimization_time,
    double information_weight, double keyframe_hz) {
  assert(landmark_container->NumImages() > 3);
  // Get matches between first and last image in the window
  const auto first_time = landmark_container->FrontTimestamp();
  const auto last_time = landmark_container->BackTimestamp();
  std::vector<Eigen::Vector2i, beam::AlignVec2i> first_landmarks;
  std::vector<Eigen::Vector2i, beam::AlignVec2i> last_landmarks;
  std::vector<uint64_t> ids =
      landmark_container->GetLandmarkIDsInImage(last_time);
  std::vector<uint64_t> matched_ids;
  for (auto& id : ids) {
    try {
      const Eigen::Vector2i first_pixel =
          landmark_container->GetValue(first_time, id).cast<int>();
      const Eigen::Vector2i last_pixel =
          landmark_container->GetValue(last_time, id).cast<int>();
      first_landmarks.push_back(first_pixel);
      last_landmarks.push_back(last_pixel);
      matched_ids.push_back(id);
    } catch (const std::out_of_range& oor) {}
  }

  // Compute relative pose between first and last
  beam::opt<Eigen::Matrix4d> T_cameraN_camera0 =
      beam_cv::RelativePoseEstimator::RANSACEstimator(
          camera_model, camera_model, first_landmarks, last_landmarks,
          beam_cv::EstimatorMethod::SEVENPOINT, 100);

  if (!T_cameraN_camera0.has_value()) { return {}; }

  Eigen::Matrix4d T_world_camera0 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_world_cameraN =
      beam::InvertTransform(T_cameraN_camera0.value());

  Eigen::Matrix4d T_camera0_world = beam::InvertTransform(T_world_camera0);
  Eigen::Matrix4d T_cameraN_world = beam::InvertTransform(T_world_cameraN);
  // Triangulate points
  auto points = beam_cv::Triangulation::TriangulatePoints(
      camera_model, camera_model, T_camera0_world, T_cameraN_world,
      first_landmarks, last_landmarks);

  // Determine inliers of triangulated points
  std::vector<int> inlier_indices;
  int total_size = first_landmarks.size();
  for (size_t i = 0; i < points.size(); i++) {
    if (!points[i].has_value()) { continue; }
    // transform points into each camera frame
    Eigen::Vector3d t_point_camera0 =
        (T_camera0_world * points[i].value().homogeneous()).hnormalized();
    Eigen::Vector3d t_point_cameraN =
        (T_cameraN_world * points[i].value().homogeneous()).hnormalized();
    // reproject triangulated points into each frame
    Eigen::Vector2d t_rep0, t_repN;
    if (!camera_model->ProjectPoint(t_point_camera0, t_rep0) ||
        !camera_model->ProjectPoint(t_point_cameraN, t_repN)) {
      total_size--;
      continue;
    }
    // compute distance to actual pixel
    Eigen::Vector2d pixel0 = first_landmarks[i].cast<double>();
    Eigen::Vector2d pixelN = last_landmarks[i].cast<double>();
    if (beam::distance(t_rep0, pixel0) < 10.0 &&
        beam::distance(t_repN, pixelN) < 10.0) {
      inlier_indices.push_back(i);
    }
  }

  // If not enough inliers, return
  float inlier_ratio = (float)inlier_indices.size() / first_landmarks.size();
  if (inlier_ratio < 0.8) {
    ROS_WARN_STREAM(__func__ << ": Invalid image pair. Inlier Ratio: "
                             << inlier_ratio);
    return {};
  }
  ROS_INFO_STREAM(__func__ << ": Valid image pair. Inlier Ratio: "
                           << inlier_ratio
                           << ". Attempting Visual Path Estimation.");

  // Perform SFM
  auto visual_graph = std::make_shared<fuse_graphs::HashGraph>();
  auto visual_map = std::make_shared<vision::VisualMap>(
      "VisionOnlySFM", camera_model, loss_function, information_weight);

  // Add initial poses, landmarks and constraints to graph
  auto initial_transaction = fuse_core::Transaction::make_shared();
  initial_transaction->stamp(first_time);

  visual_map->AddCameraPose(T_world_camera0, first_time, initial_transaction);
  visual_map->AddCameraPose(T_world_cameraN, last_time, initial_transaction);

  for (const auto& id : inlier_indices) {
    const auto landmark_id = matched_ids[id];
    visual_map->AddLandmark(points[id].value(), landmark_id,
                            initial_transaction);
    try {
      Eigen::Vector2d first_pixel =
          landmark_container->GetValue(first_time, landmark_id);
      Eigen::Vector2d last_pixel =
          landmark_container->GetValue(last_time, landmark_id);
      visual_map->AddVisualConstraint(first_time, landmark_id, first_pixel,
                                      initial_transaction);
      visual_map->AddVisualConstraint(last_time, landmark_id, last_pixel,
                                      initial_transaction);
    } catch (const std::out_of_range& oor) { continue; }
  }

  visual_graph->update(*initial_transaction);
  visual_map->UpdateGraph(*visual_graph);

  // Subsample image times to target hz
  const auto img_times = landmark_container->GetMeasurementTimesVector();
  double keyframe_period = 1.0 / keyframe_hz;
  std::vector<ros::Time> keyframe_times;
  ros::Time prev_kf(0.0);
  for (const auto& stamp : img_times) {
    if ((stamp - prev_kf).toSec() > keyframe_period) {
      keyframe_times.push_back(stamp);
      prev_kf = stamp;
    }
  }

  // process each keyframe
  for (const auto& timestamp : keyframe_times) {
    if (timestamp == first_time || timestamp == last_time) continue;

    auto transaction = fuse_core::Transaction::make_shared();
    transaction->stamp(timestamp);
    // get 2d-3d correspondences
    std::vector<Eigen::Vector2i, beam::AlignVec2i> pixels;
    std::vector<Eigen::Vector3d, beam::AlignVec3d> points;
    std::vector<uint64_t> ids_in_frame;
    for (auto& id : matched_ids) {
      bs_variables::Point3DLandmark::SharedPtr lm = visual_map->GetLandmark(id);
      if (lm) {
        try {
          Eigen::Vector3d point = lm->point();
          Eigen::Vector2i pixeli =
              landmark_container->GetValue(timestamp, id).cast<int>();
          pixels.push_back(pixeli);
          points.push_back(point);
          ids_in_frame.push_back(id);
        } catch (const std::out_of_range& oor) {}
      }
    }
    // localize with correspondences
    Eigen::Matrix4d T_CAMERA_WORLD_est =
        beam_cv::AbsolutePoseEstimator::RANSACEstimator(camera_model, pixels,
                                                        points, 100);

    // add pose to graph
    visual_map->AddCameraPose(beam::InvertTransform(T_CAMERA_WORLD_est),
                              timestamp, transaction);

    // add visual constraints to
    for (auto& id : ids_in_frame) {
      try {
        Eigen::Vector2d measurement =
            landmark_container->GetValue(timestamp, id);

        visual_map->AddVisualConstraint(timestamp, id, measurement,
                                        transaction);
      } catch (const std::out_of_range& oor) { continue; }
    }
    // add transaction to graph and notify visual map
    visual_graph->update(*transaction);
    visual_map->UpdateGraph(*visual_graph);
  }

  // optimize graph
  ROS_INFO_STREAM(__func__ << ": Optimizing trajectory.");
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 6;
  options.logging_type = ceres::PER_MINIMIZER_ITERATION;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  visual_graph->optimizeFor(ros::Duration(max_optimization_time), options);
  visual_map->UpdateGraph(*visual_graph);

  // return result
  std::map<uint64_t, Eigen::Matrix4d> init_path;
  for (const auto& timestamp : landmark_container->GetMeasurementTimes()) {
    const auto pose = visual_map->GetBaselinkPose(timestamp);
    if (pose.has_value()) { init_path[timestamp.toNSec()] = pose.value(); }
  }

  return init_path;
}

}} // namespace bs_models::vision