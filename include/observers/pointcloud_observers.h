#ifndef ARTSLAM_3D_LASER_POINTCLOUD_OBSERVERS_H
#define ARTSLAM_3D_LASER_POINTCLOUD_OBSERVERS_H

#include <artslam_types/types.hpp>
#include <pcl/point_cloud.h>

using namespace artslam::core::types;

namespace artslam::laser3d {
    class RawPointcloudObserver {
    public:
        virtual void update_raw_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) = 0;
    };

    class FilteredPointcloudObserver {
    public:
        virtual void update_filtered_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) = 0;
    };
}

#endif //ARTSLAM_3D_LASER_POINTCLOUD_OBSERVERS_H
