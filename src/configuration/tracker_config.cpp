/** @file tracker_config.cpp
 * @brief Definition of class TrackerConfig
 * @author Matteo Frosi
 */

#include <tracker_config.h>

/* ------------------ */
/* Class constructor. */
/* ------------------ */
TrackerConfig::TrackerConfig() {
    this->downsample_method = "VOXELGRID";
    this->downsample_resolution = 0.1;
    this->keyframe_delta_trans = 0.25;
    this->keyframe_delta_angle = 0.15;
    this->keyframe_delta_time = 1.0;
    this->transform_thresholding = false;
    this->max_acceptable_trans = 1.0;
    this->max_acceptable_angle = 1.0;
    this->num_frames_to_skip = 3;
    this->prior_odom_delta_trans = 0.2; // smaller than keyframe_delta_trans
    this->prior_odom_delta_angle = 0.1; // smaller than keyframe_delta_angle
}