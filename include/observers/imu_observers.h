#ifndef ARTSLAM_3D_LASER_IMU_OBSERVERS_H
#define ARTSLAM_3D_LASER_IMU_OBSERVERS_H

#include <artslam_types/types_slam.hpp>

using namespace artslam::core::types;

namespace artslam::laser3d {
    class IMUObserver {
    public:
        virtual void update_raw_imu_observer(const IMU3D_MSG::ConstPtr& imu3d_msg) = 0;
    };
}

#endif //ARTSLAM_3D_LASER_IMU_OBSERVERS_H
