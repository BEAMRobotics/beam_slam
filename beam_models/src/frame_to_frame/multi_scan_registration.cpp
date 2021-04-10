#include <beam_models/frame_to_frame/multi_scan_registration.h>

#include <beam_common/sensor_proc.h>
#include <beam_matching/Matchers.h>
#include <fuse_constraints/absolute_pose_3d_stamped_constraint.h>
#include <fuse_core/transaction.h>

namespace beam_models { namespace frame_to_frame {

MultiScanRegistration::MultiScanRegistration(
    std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher,
    int num_neighbors, double outlier_threshold_t, double outlier_threshold_r,
    const std::string& source, bool fix_first_scan)
    : matcher_(std::move(matcher)),
      num_neighbors_(num_neighbors),
      outlier_threshold_t_(outlier_threshold_t),
      outlier_threshold_r_(outlier_threshold_r),
      source_(source),
      fix_first_scan_(fix_first_scan) {
  // if outputting results, clear output folder:
  if (!output_scan_registration_results_) { return; }

  coord_frame_ = boost::make_shared<PointCloudCol>();
  *coord_frame_ = beam::CreateFrameCol();

  if (boost::filesystem::is_directory(tmp_output_path_)) {
    boost::filesystem::remove_all(tmp_output_path_);
  }
  boost::filesystem::create_directory(tmp_output_path_);
}

void MultiScanRegistration::SetFixedCovariance(
    const Eigen::Matrix<double, 6, 6>& covariance) {
  covariance_ = covariance;
  use_fixed_covariance_ = true;
}

fuse_core::Transaction::SharedPtr MultiScanRegistration::RegisterNewScan(
    const std::shared_ptr<ScanPose>& new_scan) {
  // Create a transaction object
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(new_scan->Stamp());
  transaction->addVariable(new_scan->Position(), true);
  transaction->addVariable(new_scan->Orientation(), true);
  transaction->addInvolvedStamp(new_scan->Stamp());

  // if first scan, add to list then exit
  if (reference_clouds_.empty()) {
    reference_clouds_.push_front(new_scan);
    if (fix_first_scan_) { AddPrior(new_scan, transaction); }
    return transaction;
  }

  if (output_scan_registration_results_) {
    current_scan_path_ =
        tmp_output_path_ + std::to_string(new_scan->Stamp().toSec()) + "/";
    boost::filesystem::create_directory(current_scan_path_);
  }

  int counter = 0;
  int num_constraints = 0;
  for (auto ref_iter = reference_clouds_.begin();
       ref_iter != reference_clouds_.end(); ref_iter++) {
    counter++;
    ROS_DEBUG("Matching against neighbor no. %d", counter);

    // run matcher to get refined cloud pose
    Eigen::Matrix4d T_CLOUDREF_CLOUDCURRENT;
    Eigen::Matrix<double, 6, 6> covariance;
    if (!MatchScans(*ref_iter, new_scan, T_CLOUDREF_CLOUDCURRENT, covariance)) {
      continue;
    }

    // Convert measurement to fuse variable
    Eigen::Matrix3d R = T_CLOUDREF_CLOUDCURRENT.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);
    fuse_core::Vector7d pose_relative_mean;
    pose_relative_mean << T_CLOUDREF_CLOUDCURRENT(0, 3),
        T_CLOUDREF_CLOUDCURRENT(1, 3), T_CLOUDREF_CLOUDCURRENT(2, 3), q.w(),
        q.x(), q.y(), q.z();

    // create and add constraint
    auto constraint =
        fuse_constraints::RelativePose3DStampedConstraint::make_shared(
            source_, *(*ref_iter)->Position(), *(*ref_iter)->Orientation(),
            *(new_scan->Position()), *(new_scan->Orientation()),
            pose_relative_mean, covariance);

    transaction->addConstraint(constraint, true);
    num_constraints++;
  }

  // if no constraints were added for this scan, then remove don't add variable
  // or constraint
  if (num_constraints == 0) { return nullptr; }

  // add cloud to reference cloud list and remove last
  if (reference_clouds_.size() == num_neighbors_) {
    reference_clouds_.pop_back();
  }
  reference_clouds_.push_front(new_scan);

  return transaction;
}

bool MultiScanRegistration::PassedThreshold(
    const Eigen::Matrix4d& T_measured, const Eigen::Matrix4d& T_estimated) {
  Eigen::Matrix3d R1 = T_measured.block(0, 0, 3, 3);
  Eigen::Matrix3d R2 = T_estimated.block(0, 0, 3, 3);

  double t_error =
      (T_measured.block(0, 3, 3, 1) - T_estimated.block(0, 3, 3, 1)).norm();
  double r_error = std::abs(Eigen::AngleAxis<double>(R1).angle() -
                            Eigen::AngleAxis<double>(R2).angle());

  if (t_error > outlier_threshold_t_ || r_error > outlier_threshold_r_) {
    return false;
  }
  return true;
}

void MultiScanRegistration::UpdateScanPoses(
    fuse_core::Graph::ConstSharedPtr graph_msg) {
  for (auto iter = reference_clouds_.begin(); iter != reference_clouds_.end();
       iter++) {
    (*iter)->Update(graph_msg);
  }
}

bool MultiScanRegistration::MatchScans(
    const std::shared_ptr<ScanPose>& scan_pose_1,
    const std::shared_ptr<ScanPose>& scan_pose_2,
    Eigen::Matrix4d& T_CLOUD1_CLOUD2, Eigen::Matrix<double, 6, 6>& covariance) {
  PointCloudPtr cloud1 = scan_pose_1->Cloud();
  PointCloudPtr cloud2 = scan_pose_2->Cloud();
  Eigen::Matrix4d T_WORLD_CLOUD1 = scan_pose_1->T_WORLD_CLOUD();
  Eigen::Matrix4d T_WORLD_CLOUD2 = scan_pose_2->T_WORLD_CLOUD();

  Eigen::Matrix4d T_CLOUD1_CLOUD2_init =
      beam::InvertTransform(T_WORLD_CLOUD1) * T_WORLD_CLOUD2;

  // transform cloud2 into cloud1 frame
  PointCloudPtr cloud2_RefFInit = boost::make_shared<PointCloud>();
  pcl::transformPointCloud(*cloud2, *cloud2_RefFInit,
                           Eigen::Affine3d(T_CLOUD1_CLOUD2_init));

  // match clouds
  matcher_->SetRef(cloud2_RefFInit);
  matcher_->SetTarget(cloud1);
  if (!matcher_->Match()) {
    ROS_ERROR("Failed scan matching. Skipping measurement.");
    return false;
  }

  Eigen::Matrix4d T_CLOUD1Est_CLOUD1Ini = matcher_->GetResult().matrix();
  T_CLOUD1_CLOUD2 = T_CLOUD1Est_CLOUD1Ini * T_CLOUD1_CLOUD2_init;

  if (output_scan_registration_results_) {
    PointCloudPtr cloud_ref = cloud1;
    PointCloudPtr cloud_ref_world = boost::make_shared<PointCloud>();
    PointCloudPtr cloud_cur_initial_world = boost::make_shared<PointCloud>();
    PointCloudPtr cloud_cur_aligned_world = boost::make_shared<PointCloud>();

    const Eigen::Matrix4d& T_WORLD_CLOUDCURRENT_INIT = T_WORLD_CLOUD2;
    const Eigen::Matrix4d& T_WORLD_CLOUDREF_INIT = T_WORLD_CLOUD1;
    Eigen::Matrix4d T_WORLD_CLOUDCURRENT_OPT =
        T_WORLD_CLOUDREF_INIT * T_CLOUD1_CLOUD2;

    pcl::transformPointCloud(*cloud_ref, *cloud_ref_world,
                             T_WORLD_CLOUDREF_INIT);
    pcl::transformPointCloud(*cloud2, *cloud_cur_initial_world,
                             T_WORLD_CLOUDCURRENT_INIT);
    pcl::transformPointCloud(*cloud2, *cloud_cur_aligned_world,
                             T_WORLD_CLOUDCURRENT_OPT);

    PointCloudColPtr cloud_ref_world_col =
        beam::ColorPointCloud(cloud_ref_world, 0, 0, 255);
    PointCloudColPtr cloud_cur_initial_world_col =
        beam::ColorPointCloud(cloud_cur_initial_world, 255, 0, 0);
    PointCloudColPtr cloud_cur_aligned_world_col =
        beam::ColorPointCloud(cloud_cur_aligned_world, 0, 255, 0);

    cloud_ref_world_col = beam::AddFrameToCloud(
        cloud_ref_world_col, coord_frame_, T_WORLD_CLOUDREF_INIT);
    cloud_cur_initial_world_col = beam::AddFrameToCloud(
        cloud_cur_initial_world_col, coord_frame_, T_WORLD_CLOUDCURRENT_INIT);
    cloud_cur_aligned_world_col = beam::AddFrameToCloud(
        cloud_cur_aligned_world_col, coord_frame_, T_WORLD_CLOUDCURRENT_OPT);

    std::string filename = current_scan_path_ +
                           std::to_string(scan_pose_1->Stamp().toSec()) + "U" +
                           std::to_string(scan_pose_1->Updates());

    pcl::io::savePCDFileASCII(filename + "_ref.pcd", *cloud_ref_world_col);
    pcl::io::savePCDFileASCII(filename + "_cur_init.pcd",
                              *cloud_cur_initial_world_col);
    pcl::io::savePCDFileASCII(filename + "_cur_alig.pcd",
                              *cloud_cur_aligned_world_col);

    ROS_INFO("Saved scan registration results to %s", filename.c_str());
  }

  if (!PassedThreshold(T_CLOUD1_CLOUD2,
                       beam::InvertTransform(scan_pose_1->T_WORLD_CLOUD()) *
                           scan_pose_2->T_WORLD_CLOUD())) {
    ROS_ERROR("Failed scan matcher transform threshold check. Skipping "
              "measurement.");
    return false;
  }

  if (use_fixed_covariance_) {
    covariance = covariance_;
  } else {
    matcher_->EstimateInfo();
    covariance = matcher_->GetInfo();
  }

  return true;
}

std::shared_ptr<ScanPose> MultiScanRegistration::GetScan(const ros::Time& t) {
  for (auto iter = Begin(); iter != End(); iter++) {
    if ((*iter)->Stamp() == t) { return *iter; }
  }
  return nullptr;
}

void MultiScanRegistration::PrintScanDetails(std::ostream& stream) {
  for (auto iter = Begin(); iter != End(); iter++) { (*iter)->Print(stream); }
}

void MultiScanRegistration::AddPrior(
    const std::shared_ptr<ScanPose>& scan,
    fuse_core::Transaction::SharedPtr transaction) {
  fuse_core::Vector7d mean;
  mean << scan->Position()->x(), scan->Position()->y(), scan->Position()->z(),
      scan->Orientation()->w(), scan->Orientation()->x(),
      scan->Orientation()->y(), scan->Orientation()->z();
  fuse_core::Matrix6d prior_covariance;
  prior_covariance.setIdentity();
  prior_covariance = prior_covariance * 0.0000000001;
  auto prior =
      std::make_shared<fuse_constraints::AbsolutePose3DStampedConstraint>(
          "FIRST_SCAN_PRIOR", *scan->Position(), *scan->Orientation(), mean,
          prior_covariance);
  transaction->addConstraint(prior, true);
}

}} // namespace beam_models::frame_to_frame
