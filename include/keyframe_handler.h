//
// Created by matteo on 17/11/20.
//

#ifndef ARTSLAM_KEYFRAME_HANDLER_H
#define ARTSLAM_KEYFRAME_HANDLER_H


#include <Eigen/Dense>

class KeyframeHandler {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyframeHandler();
    KeyframeHandler(double delta_trans, double delta_angle, double delta_time);

    bool is_frame_key(const Eigen::Isometry3d& pose, uint64_t timestamp);

    double get_accum_distance() const;

private:
    double keyframe_delta_trans;
    double keyframe_delta_angle;
    double keyframe_delta_time;

    bool is_first;
    double accum_distance;
    Eigen::Isometry3d prev_keypose;
    uint64_t prev_timestamp;
};


#endif //ARTSLAM_KEYFRAME_HANDLER_H
