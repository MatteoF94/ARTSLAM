#ifndef ARTSLAM_LASER_3D_PRETRACKER_H
#define ARTSLAM_LASER_3D_PRETRACKER_H

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/registration.h>
#include <artslam_utils/dispatcher.h>
#include <artslam_types/types_pcl.hpp>
#include "observers/pointcloud_observers.h"
#include "observers/odometry_observers.h"

using namespace artslam::core::types;
using namespace artslam::core::utils;

namespace artslam::laser3d {
    class Pretracker : public RawPointcloudObserver {
    public:
        struct Configuration {
            bool medium_scale_enabled_ = false;         // whether medium scale tracking is enabled or not
            float low_scale_leaf_size_ = 0.75;
            float medium_scale_leaf_size_ = 0.5;
            bool verbose_ = true;
        };

        Pretracker(pcl::Registration<Point3I,Point3I>::Ptr lsrm, pcl::Registration<Point3I,Point3I>::Ptr msrm);

        Pretracker(const Configuration& configuration, pcl::Registration<Point3I, Point3I>::Ptr lsrm, pcl::Registration<Point3I, Point3I>::Ptr msrm);

        // Signals that a new raw point cloud has been received
        void update_raw_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) override;

        // Registers an object waiting for odometries
        void register_odometry_observer(PriorOdometryObserver* odometry_observer);

        // Removes an object waiting for odometries
        void remove_odometry_observer(PriorOdometryObserver* odometry_observer);

    private:
        void pretrack(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        void handle_first_frame(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        EigMatrix4f pretrack_low_scale(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        EigMatrix4f pretrack_medium_scale(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud, const EigMatrix4f& guess);

        void notify_odometry_observers(const OdometryStamped3D_MSG::ConstPtr& odometry3d_msg);

        bool first_frame_ = true;                   // whether the input point cloud is the first received
        bool medium_scale_enabled_ = false;         // whether medium scale tracking is enabled or not

        EigMatrix4f odometry_ = EigMatrix4f::Identity();

        pcl::Registration<Point3I, Point3I>::Ptr low_scale_registration_method_;
        pcl::Registration<Point3I, Point3I>::Ptr medium_scale_registration_method_;

        pcl::PointCloud<Point3I>::Ptr low_scale_pointcloud_;
        pcl::PointCloud<Point3I>::Ptr medium_scale_pointcloud_;

        pcl::VoxelGrid<Point3I> low_scale_downsampler_;
        pcl::VoxelGrid<Point3I> medium_scale_downsampler_;
        float low_scale_leaf_size_ = 0.75;
        float medium_scale_leaf_size_ = 0.5;

        std::unique_ptr<Dispatcher> pretracker_dispatcher_;     // dispatcher of the tracking events
        std::vector<PriorOdometryObserver*> odometry_observers_;     // vector of odometry observers

        bool verbose_ = false;
    };
}

#endif //ARTSLAM_LASER_3D_PRETRACKER_H
