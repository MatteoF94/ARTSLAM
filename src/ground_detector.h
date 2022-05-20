#ifndef ARTSLAM_LASER_3D_GROUND_DETECTOR_H
#define ARTSLAM_LASER_3D_GROUND_DETECTOR_H

#include <artslam_types/types_pcl.hpp>
#include <artslam_utils/dispatcher.h>
#include "observers/pointcloud_observers.h"
#include "observers/floor_coefficients_observers.h"

using namespace artslam::core::types;
using namespace artslam::core::utils;

namespace artslam::laser3d {
    class GroundDetector : public FilteredPointcloudObserver {
    public:
        // Defines the configuration parameters of the class
        struct Configuration {
            float tilt_angle_ = 0.0;                    // tilt angle of the sensor w.r.t. the ground plane
            float sensor_height_ = 2.0;                 // rough estimate of the sensor height
            float clipping_range_ = 0.5;                // -[sensor_height_ -+ clipping_range_] is the range in which search the floor
            int floor_points_threshold_ = 512;          // minimum number of points to detect ground coefficients
            bool use_normal_filtering_ = false;         // whether to remove points with inclined normals
            float normal_filtering_threshold_ = 0.5;    // maximum allowed angle formed by point normal and z axis
            bool rough_ground_check_ = false;           // whether to check for rough terrains, in case normal search fails
            float rough_ground_max_dist_ = 3.0;         // maximum distance from sensor
            bool verbose_ = false;                       // whether the class should be verbose
        };

        // Class constructor
        GroundDetector();

        // Class constructor, with parameters
        explicit GroundDetector(const Configuration& configuration);

        // Signals that a new filtered point cloud has been received
        void update_filtered_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) override;

        // Registers an object waiting for floor coefficients
        void register_floor_coefficients_observer(FloorCoefficientsObserver* floor_coefficients_observer);

        // Removes an object waiting for floor coefficients
        void remove_floor_coefficients_observer(FloorCoefficientsObserver* floor_coefficients_observer);

        // testing purposes
        uint64_t total_time_ = 0;
        uint32_t count_ = 0;

    private:
        void detect_ground_model(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        pcl::PointCloud<Point3I>::Ptr filter_pointcloud(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        pcl::PointCloud<Point3I>::Ptr clip_with_plane(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud, const EigVector4f& plane, bool negative) const;

        pcl::PointCloud<Point3I>::Ptr filter_normals(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) const;

        std::optional<EigVector4d> detect_floor_coefficients(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) const;

        std::optional<EigVector4d> detect_rough_ground_coefficients(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) const;

        // Notifies an object waiting for floor coefficients
        void notify_floor_coefficients_observers(const FloorCoefficients_MSG::ConstPtr& floor_coefficients_msg);

        // ----------------------------------------------------------------------------------
        // ---------------------------- PARAMETERS AND VARIABLES ----------------------------
        // ----------------------------------------------------------------------------------
        Configuration configuration_;                                           // configuration object
        std::unique_ptr<Dispatcher> ground_detector_dispatcher_;                // core operations handler
        std::vector<FloorCoefficientsObserver*> floor_coefficients_observers_;  // modules which need floor coefficients
    };
}


#endif //ARTSLAM_LASER_3D_GROUND_DETECTOR_H
