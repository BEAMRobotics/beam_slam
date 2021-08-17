#include <bs_models/camera_to_camera/utils.h>

namespace bs_models { namespace camera_to_camera {

std::shared_ptr<beam_cv::PoseRefinement> PoseRefiner() {
  ceres::Solver::Options pose_refinement_options;
  pose_refinement_options.minimizer_progress_to_stdout = false;
  pose_refinement_options.logging_type = ceres::SILENT;
  pose_refinement_options.max_solver_time_in_seconds = 1e-3;
  pose_refinement_options.function_tolerance = 1e-4;
  pose_refinement_options.gradient_tolerance = 1e-6;
  pose_refinement_options.parameter_tolerance = 1e-4;
  pose_refinement_options.linear_solver_type = ceres::SPARSE_SCHUR;
  pose_refinement_options.preconditioner_type = ceres::SCHUR_JACOBI;
  std::shared_ptr<beam_cv::PoseRefinement> pose_refiner =
      std::make_shared<beam_cv::PoseRefinement>(pose_refinement_options);
  return pose_refiner;
}

beam::opt<Eigen::Matrix4d> LocalizeFrame(
    const std::shared_ptr<beam_cv::Tracker>& tracker,
    const std::shared_ptr<bs_models::camera_to_camera::VisualMap>& visual_map,
    const std::shared_ptr<beam_cv::PoseRefinement>& pose_refiner,
    const std::shared_ptr<beam_calibration::CameraModel>& cam_model,
    const ros::Time& frame_time) {
  std::vector<Eigen::Vector2i, beam_cv::AlignVec2i> pixels;
  std::vector<Eigen::Vector3d, beam_cv::AlignVec3d> points;
  std::vector<uint64_t> landmarks = tracker->GetLandmarkIDsInImage(frame_time);
  // get 2d-3d correspondences
  for (auto& id : landmarks) {
    fuse_variables::Point3DLandmark::SharedPtr lm = visual_map->GetLandmark(id);
    if (lm) {
      Eigen::Vector2i pixeli = tracker->Get(frame_time, id).cast<int>();
      pixels.push_back(pixeli);
      Eigen::Vector3d point(lm->x(), lm->y(), lm->z());
      points.push_back(point);
    }
  }
  // perform ransac pnp for initial estimate
  if (points.size() >= 15) {
    Eigen::Matrix4d T_CAMERA_WORLD_est =
        beam_cv::AbsolutePoseEstimator::RANSACEstimator(cam_model, pixels,
                                                        points);
    // refine pose using motion only BA
    Eigen::Matrix4d T_CAMERA_WORLD_ref =
        pose_refiner->RefinePose(T_CAMERA_WORLD_est, cam_model, pixels, points);
    Eigen::Matrix4d T_WORLD_CAMERA = T_CAMERA_WORLD_ref.inverse();
    return T_WORLD_CAMERA;
  } else {
    return {};
  }
}

std::map<uint64_t, Eigen::Vector3d> MatchFrameToCurrentSubmap(
    const std::shared_ptr<beam_cv::Tracker>& tracker,
    const std::shared_ptr<bs_models::camera_to_camera::VisualMap>& visual_map,
    const std::shared_ptr<beam_calibration::CameraModel>& cam_model,
    const ros::Time& frame_time) {
  // vector of poitns to return
  std::map<uint64_t, Eigen::Vector3d> matched_points;
  bs_common::Submap& submap = bs_common::Submap::GetInstance();
  // get map points in current camera frame
  Eigen::Matrix4d T_WORLD_CAMERA = visual_map->GetPose(frame_time).value();
  std::vector<Eigen::Vector3d> points_camera =
      submap.GetVisualMapPoints(T_WORLD_CAMERA);
  std::vector<cv::Mat> descriptors = submap.GetDescriptors();

  // if submap empty then true empty vector
  if (points_camera.size() == 0) { return matched_points; }

  std::vector<cv::KeyPoint> projected_keypoints, current_keypoints;
  cv::Mat projected_descriptors, current_descriptors;
  // project each point to get keypoints
  for (size_t i = 0; i < points_camera.size(); i++) {
    Eigen::Vector3d point = points_camera[i];
    // project point into image plane
    bool in_image = false;
    Eigen::Vector2d pixel;
    cam_model->ProjectPoint(point, pixel, in_image);
    // make cv keypoint for pixel
    cv::KeyPoint kp;
    kp.pt.x = (float)pixel[0];
    kp.pt.y = (float)pixel[1];
    // get descriptor
    cv::Mat desc = descriptors[i];
    // push to results
    projected_descriptors.push_back(desc);
    projected_keypoints.push_back(kp);
  }
  std::vector<uint64_t> untriangulated_ids;
  std::vector<uint64_t> landmarks = tracker->GetLandmarkIDsInImage(frame_time);
  for (auto& id : landmarks) {
    if (!visual_map->GetLandmark(id) && !visual_map->GetFixedLandmark(id)) {
      Eigen::Vector2d pixel = tracker->Get(frame_time, id);
      cv::KeyPoint kp;
      kp.pt.x = (float)pixel[0];
      kp.pt.y = (float)pixel[1];
      cv::Mat desc = tracker->GetDescriptor(frame_time, id);
      current_descriptors.push_back(desc);
      current_keypoints.push_back(kp);
    }
  }

  // match features
  std::shared_ptr<beam_cv::Matcher> matcher =
      std::make_shared<beam_cv::BFMatcher>(cv::NORM_HAMMING);
  std::vector<cv::DMatch> matches =
      matcher->MatchDescriptors(projected_descriptors, current_descriptors,
                                projected_keypoints, current_keypoints);
  // filter matches by pixel distance
  for (auto& m : matches) {
    Eigen::Vector2d kp1 =
        beam_cv::ConvertKeypoint(projected_keypoints[m.queryIdx]);
    Eigen::Vector2d kp2 =
        beam_cv::ConvertKeypoint(current_keypoints[m.trainIdx]);
    // if the match is less than pixel threshold then add point to return map
    if (beam::distance(kp1, kp2) < 20) {
      Eigen::Vector3d p_camera = points_camera[m.queryIdx];
      // transform point back into world frame
      Eigen::Vector4d ph_camera{p_camera[0], p_camera[1], p_camera[2], 1};
      Eigen::Vector3d p_world = (T_WORLD_CAMERA * ph_camera).hnormalized();
      // remove match from submap
      submap.RemoveVisualMapPoint(m.queryIdx);
      matched_points[untriangulated_ids[m.queryIdx]] = p_world;
    }
  }
  return matched_points;
}

double ComputeAvgParallax(const std::shared_ptr<beam_cv::Tracker>& tracker,
                          const ros::Time& t1, const ros::Time& t2) {
  std::vector<uint64_t> ids1 = tracker->GetLandmarkIDsInImage(t1);
  // add parallaxes to vector
  std::vector<double> parallaxes;
  for (auto& id : ids1) {
    try {
      Eigen::Vector2d p1 = tracker->Get(t1, id);
      Eigen::Vector2d p2 = tracker->Get(t2, id);
      double dist = beam::distance(p1, p2);
      parallaxes.push_back(dist);
    } catch (const std::out_of_range& oor) {}
  }
  // sort and find median parallax
  std::sort(parallaxes.begin(), parallaxes.end());
  return parallaxes[parallaxes.size() / 2];
}

}} // namespace bs_models::camera_to_camera