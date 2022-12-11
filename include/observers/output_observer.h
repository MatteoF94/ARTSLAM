#ifndef ARTSLAM_LASER_3D_WRAPPER_OUTPUT_OBSERVER_H
#define ARTSLAM_LASER_3D_WRAPPER_OUTPUT_OBSERVER_H

#include <artslam_types/types.hpp>
#include <pcl/point_cloud.h>

using namespace artslam::core::types;

namespace artslam::laser3d {
    class SlamOutputObserver {
    public:
        virtual void update_slam_output_observer(pcl::PointCloud<Point3I>::Ptr map, std::vector<Eigen::Isometry3d> poses, OccupancyGrid::Ptr oc) = 0;
    };
}

#endif //ARTSLAM_LASER_3D_WRAPPER_OUTPUT_OBSERVER_H
