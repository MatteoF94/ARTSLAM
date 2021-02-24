/** @file floor_detector_config.cpp
 * @brief Definition of class FloorDetectorConfig
 */

#include <floor_detector_config.h>

FloorDetectorConfig::FloorDetectorConfig() {
    this->tilt_deg = 0.0;
    this->sensor_height = 2.0;
    this->height_clip_range = 1.0;
    this->floor_pts_thresh = 512;
    this->floor_normal_thresh = 10.0;
    this->use_normal_filtering = true;
    this->normal_filter_thresh = 20.0;
}