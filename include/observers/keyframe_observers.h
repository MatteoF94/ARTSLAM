#ifndef ARTSLAM_LASER_3D_KEYFRAME_OBSERVERS_H
#define ARTSLAM_LASER_3D_KEYFRAME_OBSERVERS_H

#include "keyframe_laser_3d.h"

using namespace artslam::core::types;

namespace artslam::laser3d {
    class KeyframeObserver {
    public:
        virtual void update_keyframe_observer(const KeyframeLaser3D::Ptr& keyframe) = 0;
    };
}

#endif //ARTSLAM_LASER_3D_KEYFRAME_OBSERVERS_H
