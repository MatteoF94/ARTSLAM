#ifndef ARTSLAM_LASER_3D_GNSS_OBSERVERS_H
#define ARTSLAM_LASER_3D_GNSS_OBSERVERS_H

#include <artslam_types/types_slam.hpp>

using namespace artslam::core::types;

namespace artslam::laser3d {
    class GNSSObserver {
    public:
        virtual void update_raw_gnss_observer(const GeoPointStamped_MSG::ConstPtr& gnss_msg) = 0;
    };
}

#endif //ARTSLAM_LASER_3D_GNSS_OBSERVERS_H
