#pragma once

#include <map>

#include <fuse_graphs/hash_graph.h>

#include <bs_models/global_mapping/submap.h>

namespace bs_models::global_mapping {

struct RegistrationResult {
  RegistrationResult(const Eigen::Matrix4d& Ti, const Eigen::Matrix4d& Tf);
  double dt; // mm
  double dR; // deg
};

using RegistrationResults = std::map<ros::Time, RegistrationResult>;

void SaveViewableSubmap(const std::string& save_path, const PointCloud& cloud,
                        uint8_t r, uint8_t g, uint8_t b,
                        const Eigen::Vector3d& t_world_BaselinkStart,
                        const Eigen::Vector3d& t_world_BaselinkEnd);

/**
 * @brief update all scans in a vector of submaps with a graph message. We
 iterate through submaps from first to last, and each submap we iterate from
 start to end of the lidar scans, seeing if the pose variables are in the graph.
 Once we hit the max_timestamp, then we return (to save compute)
 * @param submaps global map submaps
 * @param graph graph with poses, all poses must be in the world coordinate
 frame, NOT SUBMAP FRAME (which lidar scans are normally expressed in)
 * @param max_timestamp will stop updating scans that come after this timestamp
 (for speed)
 * @param scan_poses_in_world if set to true, then all scan poses in the Submap
 are expected to be in the world frame already. If this is set to false, then we
 expect all scan poses in the Submap to be in the submap frames which is default
 for beam_slam
*/
void UpdateSubmapScanPosesFromGraph(
    std::vector<SubmapPtr> submaps,
    std::shared_ptr<fuse_graphs::HashGraph> graph, uint64_t max_timestamp = 0,
    bool scan_poses_in_world = false, bool verbose = false);

/**
 * @brief update the submap poses (i.e., first pose in the submap, or submap
 * anchor)  if the pose exists in the graph
 */
void UpdateSubmapPosesFromGraph(std::vector<SubmapPtr> submaps,
                                std::shared_ptr<fuse_graphs::HashGraph> graph,
                                bool verbose = false);

} // namespace bs_models::global_mapping