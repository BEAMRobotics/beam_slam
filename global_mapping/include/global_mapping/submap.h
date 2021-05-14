#pragma once

#include <beam_utils/pointclouds.h>

#include <boost/filesystem.hpp>
#include <fuse_core/graph.h>
#include <fuse_variables/orientation_3d_stamped.h>
#include <fuse_variables/position_3d_stamped.h>
#include <ros/time.h>

namespace global_mapping {

class Submap {
    public:

    Submap() = default;

    ~Submap() = default;

    private:
    ros::Time stamp_;
    int graph_updates_{0};
    fuse_variables::Position3DStamped t_WORLD_ANCHOR_;
    fuse_variables::Orientation3DStamped R_WORLD_ANCHOR_;
};    

} // namespace global_mapping