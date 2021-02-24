//
// Created by matteo on 17/11/20.
//

#include <keyframe_handler.h>

KeyframeHandler::KeyframeHandler() {
    this->keyframe_delta_trans = 0.25;
    this->keyframe_delta_angle = 0.15;
    this->keyframe_delta_time = 1.0;
}

KeyframeHandler::KeyframeHandler(double delta_trans, double delta_angle, double delta_time) {
    this->keyframe_delta_trans = delta_trans;
    this->keyframe_delta_angle = delta_angle;
    this->keyframe_delta_time = delta_time;
}

bool KeyframeHandler::is_frame_key(const Eigen::Isometry3d &pose, uint64_t timestamp) {
    if(this->is_first) {
        this->is_first = false;
        this->prev_keypose = pose;
        return true;
    }

    // calculate the deltas from the previous keyframes
    Eigen::Isometry3d delta = this->prev_keypose.inverse()*pose;
    double dx = delta.translation().norm();
    double da = Eigen::AngleAxisd(delta.linear()).angle();
    double dt = timestamp - this->prev_timestamp;

    if(dx < this->keyframe_delta_trans && da < this->keyframe_delta_angle && dt < this->keyframe_delta_time*1e9) {
        return false;
    }

    this->accum_distance += dx;
    this->prev_keypose = pose;
    this->prev_timestamp = timestamp;

    return true;
}

double KeyframeHandler::get_accum_distance() const {
    return this->accum_distance;
}
