#include <beam_models/frame_to_frame/multi_scan_registration.h>

#include <beam_common/sensor_proc.h>
#include <beam_matching/Matchers.h>
#include <fuse_core/transaction.h>

namespace beam_models { namespace frame_to_frame {

MultiScanRegistration::MultiScanRegistration(
    std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher,
    int num_neighbors, double outlier_threshold_t, double outlier_threshold_r,
    const std::string& source)
    : matcher_(std::move(matcher)),
      num_neighbors_(num_neighbors),
      outlier_threshold_t_(outlier_threshold_t),
      outlier_threshold_r_(outlier_threshold_r),
      source_(source) {}

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
    return transaction;
  }

  int counter = 0;
  for (auto ref_iter = reference_clouds_.begin();
       ref_iter != reference_clouds_.end(); ref_iter++) {
    counter++;
    ROS_DEBUG("Matching against neighbor no. %d", counter);

    // run matcher to get refined cloud pose
    Eigen::Matrix4d T_CLOUDREF_CLOUDCURRENT;
    Eigen::Matrix<double, 6, 6> covariance;
    MatchScans((*ref_iter)->Cloud(), new_scan->Cloud(),
               (*ref_iter)->T_WORLD_CLOUD(), new_scan->T_WORLD_CLOUD(),
               T_CLOUDREF_CLOUDCURRENT, covariance);

    if (output_scan_registration_results_) {
      PointCloudPtr cloud_ref = (*ref_iter)->Cloud();
      PointCloud cloud_ref_world;
      PointCloud cloud_cur_initial_world;
      PointCloud cloud_cur_aligned_world;

      const Eigen::Matrix4d& T_WORLD_CLOUDCURRENT_INIT =
          new_scan->T_WORLD_CLOUD();
      const Eigen::Matrix4d& T_WORLD_CLOUDREF_INIT =
          (*ref_iter)->T_WORLD_CLOUD();
      Eigen::Matrix4d T_WORLD_CLOUDCURRENT_OPT =
          T_WORLD_CLOUDREF_INIT * T_CLOUDREF_CLOUDCURRENT;

      pcl::transformPointCloud(*cloud_ref, cloud_ref_world,
                               T_WORLD_CLOUDREF_INIT);
      pcl::transformPointCloud(*new_scan->Cloud(), cloud_cur_initial_world,
                               T_WORLD_CLOUDCURRENT_INIT);
      pcl::transformPointCloud(*new_scan->Cloud(), cloud_cur_aligned_world,
                               T_WORLD_CLOUDCURRENT_OPT);

      std::string filename = tmp_output_path_ +
                             std::to_string((*ref_iter)->Stamp().toSec()) +
                             "U" + std::to_string((*ref_iter)->Updates());

      pcl::io::savePCDFileASCII(filename + "_ref.pcd", cloud_ref_world);
      pcl::io::savePCDFileASCII(filename + "_cur_init.pcd",
                                cloud_cur_initial_world);
      pcl::io::savePCDFileASCII(filename + "_cur_alig.pcd",
                                cloud_cur_aligned_world);
    }

    if (!PassedThreshold(T_CLOUDREF_CLOUDCURRENT,
                         beam::InvertTransform((*ref_iter)->T_WORLD_CLOUD()) *
                             new_scan->T_WORLD_CLOUD())) {
      ROS_DEBUG("Failed scan matcher transform threshold check. Skipping "
                "measurement.");
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
  }

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

void MultiScanRegistration::MatchScans(
    const PointCloudPtr& cloud1, const PointCloudPtr& cloud2,
    const Eigen::Matrix4d& T_WORLD_CLOUD1,
    const Eigen::Matrix4d& T_WORLD_CLOUD2, Eigen::Matrix4d& T_CLOUD1_CLOUD2,
    Eigen::Matrix<double, 6, 6>& covariance) {
  Eigen::Matrix4d T_CLOUD1_CLOUD2_init =
      beam::InvertTransform(T_WORLD_CLOUD1) * T_WORLD_CLOUD2;

  // transform cloud2 into cloud1 frame
  PointCloudPtr cloud2_RefFInit = boost::make_shared<PointCloud>();
  pcl::transformPointCloud(*cloud2, *cloud2_RefFInit,
                           Eigen::Affine3d(T_CLOUD1_CLOUD2_init));

  // match clouds
  matcher_->SetRef(cloud2_RefFInit);
  matcher_->SetTarget(cloud1);
  matcher_->Match();

  if(use_fixed_covariance_){
    covariance = covariance_;
  } else {
    matcher_->EstimateInfo();
    covariance = matcher_->GetInfo();
  }
  
  Eigen::Matrix4d T_CLOUD1Est_CLOUD1Ini = matcher_->GetResult().matrix();
  T_CLOUD1_CLOUD2 = T_CLOUD1Est_CLOUD1Ini * T_CLOUD1_CLOUD2_init;
}

}} // namespace beam_models::frame_to_frame
