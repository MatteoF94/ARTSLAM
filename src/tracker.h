#ifndef ARTSLAM_LASER_3D_TRACKER_H
#define ARTSLAM_LASER_3D_TRACKER_H

#include <pcl/registration/registration.h>
#include "observers/pointcloud_observers.h"
#include "observers/odometry_observers.h"
#include "observers/keyframe_observers.h"
#include <artslam_types/types_eigen.hpp>
#include <artslam_utils/dispatcher.h>
#include <pcl/filters/passthrough.h>
#include <keyframe_laser_3d.h>

using namespace artslam::core::types;
using namespace artslam::core::utils;

namespace artslam::laser3d {
    class Tracker : public FilteredPointcloudObserver, public PriorOdometryObserver {
    public:
        // Defines the configuration parameters of the class
        struct Configuration {
            double keyframe_delta_trans_ = 5.0;         // minimum distance (meters) between keyframes
            double keyframe_delta_angle_ = 1;           // minimum angle (radians) between keyframes
            uint64_t keyframe_delta_time_ = 1e9;        // minimum time (nanoseconds) between keyframes
            bool use_height_filter_ = true;             // whether to use a height filter on the input point cloud
            std::string filter_axis_ = "z";             // axis on which filter the height of the input point cloud
            float min_height_ = -1.6;                   // minimum acceptable height (the rest is filtered out)
            bool use_prior_odometry_ = true;            // whether to use prior odometry to speed-up tracking
            double prior_odometry_delta_trans_ = 0.5;   // maximum distance between keyframe and odometry measurement
            double prior_odometry_delta_angle_ = 0.25;  // maximum angle (radians) between keyframe and odometry measurement
            uint64_t max_odometry_delta_time_ = 1e8;    // maximum allowed time interval between an odometry measurement and the input point cloud
            EigQuaternionf initial_orientation_ = EigQuaternionf::Identity();   // initial orientation
            EigVector3f initial_translation_ = EigVector3f::Zero();             // initial translation w.r.t. the origin
            bool utm_compensated_ = false;              // whether the initial position is set (not necessarily the origin)
            bool verbose_ = true;                       // whether the class should be verbose
        };

        // Class constructor
        explicit Tracker(pcl::Registration<Point3I, Point3I>::Ptr registration_method);

        // Class constructor, with parameters
        Tracker(const Configuration& configuration, pcl::Registration<Point3I, Point3I>::Ptr registration_method);

        // Signals that a new filtered point cloud has been received
        void update_filtered_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) override;

        // Signals that a new odometry measurement has been received
        void update_prior_odometry_observer(OdometryStamped3D_MSG::ConstPtr odometry3d_msg) override;

        // Registers an object waiting for keyframes
        void register_keyframe_observer(KeyframeObserver* keyframe_observer);

        // Removes an object waiting for keyframes
        void remove_keyframe_observer(KeyframeObserver* keyframe_observer);

        // Registers an object waiting for odometries
        void register_odometry_observer(OdometryObserver* odometry_observer);

        // Removes an object waiting for odometries
        void remove_odometry_observer(OdometryObserver* odometry_observer);

        void store_guesses(const std::vector<EigIsometry3f>& guesses);

        // testing purposes
        uint64_t total_time_ = 0;
        uint32_t count_ = 0;

    private:
        void track(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        EigMatrix4f match(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        bool has_prior_odometry_guess_(uint64_t timestamp);

        // Creates a new keyframe
        KeyframeLaser3D::Ptr create_keyframe();

        bool is_odometry_updated(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud, const EigMatrix4f& odometry);

        // Notifies all the objects waiting for keyframes
        void notify_keyframe_laser3d_observers(const KeyframeLaser3D::Ptr& keyframe);

        void notify_odometry_observers(const OdometryStamped3D_MSG::ConstPtr& odometry3d_msg);

        // ----------------------------------------------------------------------------------
        // ---------------------------- PARAMETERS AND VARIABLES ----------------------------
        // ----------------------------------------------------------------------------------
        Configuration configuration_;                       // configuration object
        pcl::PassThrough<Point3I> height_filter_;  // height filter to limit a point cloud size on one axis

        bool first_frame_ = true;                   // whether the input point cloud is the first received
        bool first_keyframe_ = true;                // whether the first keyframe has already been created
        double accumulated_distance_ = 0.0;         // total accumulated translation
        bool skip_mode_ = false;                    // whether skipping scan matching is enabled
        bool enable_prior_odometry_ = true;

        std::mutex prior_odometry_mutex_;                                   // mutex used when handling odometry messages
        std::vector<OdometryStamped3DMSG_tag::ConstPtr> odometry3d_msgs_;   // vector of odometry messages
        EigMatrix4f prior_odometry_transformation_;                         // transformation between current keyframe and odometry closest in time to the input point clud

        EigMatrix4f keyframe_pose_;                                 // odometry of the current keyframe
        uint64_t keyframe_timestamp_ = 0;                           // timestamp (nanoseconds) of the current keyframe
        pcl::PointCloud<Point3I>::ConstPtr keyframe_pointcloud_;    // point cloud of the current keyframe
        double current_fitness_ = 0;                                // fitness score of the current estimated motion
        EigMatrix4f prev_transformation_;                           // transformation between keyframe and previous point cloud

        pcl::Registration<Point3I, Point3I>::Ptr registration_method_;  // algorithm used for scan matching
        std::unique_ptr<Dispatcher> tracker_dispatcher_;                // core operations handler
        std::vector<KeyframeObserver*> keyframe_observers_;             // vector of keyframe observers
        std::vector<OdometryObserver*> odometry_observers_;             // vector of odometry observers

        std::vector<EigIsometry3f> guesses_;
    };
}


#endif //ARTSLAM_LASER_3D_TRACKER_H
