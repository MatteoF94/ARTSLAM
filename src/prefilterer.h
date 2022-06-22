#ifndef ARTSLAM_3D_LASER_PREFILTERER_H
#define ARTSLAM_3D_LASER_PREFILTERER_H

#include <artslam_types/types.hpp>
#include <artslam_types/types_slam.hpp>
#include <artslam_utils/dispatcher.h>
#include <observers/pointcloud_observers.h>
#include <observers/imu_observers.h>
#include <pcl/filters/filter.h>
#include <boost/log/trivial.hpp>

using namespace artslam::core::types;
using namespace artslam::core::utils;

namespace artslam::laser3d {
    class Prefilterer : public RawPointcloudObserver, public IMUObserver {
    public:
        // Defines the configuration parameters of the class
        struct Configuration {
            std::string downsample_method_ = "VOXELGRID";
            float downsample_resolution_ = 0.2;
            std::string outlier_removal_method_ = "RADIUS";
            float radius_radius_ = 0.8;
            int radius_min_neighbours_ = 2;
            bool enable_distance_filter_ = true;
            float distance_near_threshold_ = 1.0;
            float distance_far_threshold_ = 100.0;
            bool enable_deskewing_ = false;
            float scan_period_ = 0.1;
            double imu_to_lidar_rotation_[9] = {9.999976e-01, 7.553071e-04, -2.035826e-03, -7.854027e-04, 9.998898e-01, -1.482298e-02, 2.024406e-03, 1.482454e-02, 9.998881e-01};
            bool verbose_ = true;
        };

        // Class constructor
        Prefilterer();

        // Class constructor, with parameters
        explicit Prefilterer(const Configuration& configuration);

        // Class destructor
        ~Prefilterer();

        // Signals that a new raw point cloud has been received
        void update_raw_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) override;

        // Signals that a new IMU datum has been received
        void update_raw_imu_observer(const IMU3D_MSG::ConstPtr& imu3d_msg) override;

        // Registers an object waiting for filtered point clouds
        void register_filtered_pointcloud_observer(FilteredPointcloudObserver* filtered_pointcloud_observer);

        // Removes an object waiting for filtered point clouds
        void remove_filtered_pointcloud_observer(FilteredPointcloudObserver* filtered_pointcloud_observer);

        // testing purposes
        uint64_t total_time_ = 0;
        uint32_t count_ = 0;

    private:
        // Configures the downsampling method used to filter point clouds
        void configure_downsampler(const std::string& method, float downsample_resolution);

        // Configures the outlier removal method used to filter point clouds
        void configure_outlier_remover(const std::string& method, double radius, int min_neighbours);

        // Filters and adjusts a point cloud
        void filter_pointcloud(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        // Deskews the point cloud using IMU data
        pcl::PointCloud<Point3I>::ConstPtr deskew(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        // Filters the points of the point cloud, within threshold boundaries
        pcl::PointCloud<Point3I>::ConstPtr distance_filter(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) const;

        // Downsamples the point cloud to reduce its size
        pcl::PointCloud<Point3I>::ConstPtr downsample(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        // Removes noise and outliers from the point cloud
        pcl::PointCloud<Point3I>::ConstPtr remove_outliers(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        // Notifies all the objects waiting for filtered point clouds
        void notify_filtered_pointcloud_observers(const pcl::PointCloud<Point3I>::ConstPtr& filtered_pointcloud);

        // ----------------------------------------------------------------------------------
        // ---------------------------- PARAMETERS AND VARIABLES ----------------------------
        // ----------------------------------------------------------------------------------
        Configuration configuration_;                                               // configuration object
        pcl::Filter<Point3I>::Ptr downsample_filter_;                               // filter used to downsample the cloud
        std::vector<pcl::Filter<Point3I>::Ptr> outlier_removal_filters_;            // filters used to remove outliers from the cloud
        std::vector<IMU3D_MSG::ConstPtr> imu3d_msgs_;                               // vector of IMU data (for deskewing)
        std::mutex insertion_mutex_;                                                // mutex to handle IMU data
        std::vector<FilteredPointcloudObserver*> filtered_pointcloud_observers_;    // modules which need filtered clouds
        std::unique_ptr<Dispatcher> prefilterer_dispatcher_;                        // core operations handler
    };
}


#endif //ARTSLAM_3D_LASER_PREFILTERER_H
