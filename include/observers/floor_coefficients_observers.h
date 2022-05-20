#ifndef ARTSLAM_LASER_3D_FLOOR_COEFFICIENTS_OBSERVERS_H
#define ARTSLAM_LASER_3D_FLOOR_COEFFICIENTS_OBSERVERS_H

#include <artslam_types/types_slam.hpp>

using namespace artslam::core::types;

namespace artslam::laser3d {
    class FloorCoefficientsObserver {
    public:
        virtual void update_floor_coefficients_observer(const FloorCoefficients_MSG::ConstPtr& floor_coefficients_msg) = 0;
    };
}

#endif //ARTSLAM_LASER_3D_FLOOR_COEFFICIENTS_OBSERVERS_H
