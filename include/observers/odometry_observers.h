#ifndef ARTSLAM_LASER_3D_ODOMETRY_OBSERVERS_H
#define ARTSLAM_LASER_3D_ODOMETRY_OBSERVERS_H

#include <artslam_types/types_slam.hpp>

using namespace artslam::core::types;

namespace artslam::laser3d {
    class OdometryObserver {
    public:
        virtual void update_odometry_observer(OdometryStamped3D_MSG::ConstPtr odometry_3d_msg) = 0;
    };

    class PriorOdometryObserver {
    public:
        virtual void update_prior_odometry_observer(OdometryStamped3D_MSG::ConstPtr odometry_3d_msg) = 0;
    };
}

#endif //ARTSLAM_LASER_3D_ODOMETRY_OBSERVERS_H
