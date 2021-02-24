/** @file loop_detector_config.cpp
 * @brief Definition of class LoopDetectorConfig
 * @author Matteo Frosi
 */

#include <loop_detector_config.h>
#include <limits>

/**
 * @brief Class constructor.
 */
LoopDetectorConfig::LoopDetectorConfig() {
    this->distance_thresh = 35.0;
    this->accum_distance_thresh = 25.0;
    this->distance_from_last_edge_thresh = 15.0;

    this->fitness_score_max_range = std::numeric_limits<double>::max();
    this->fitness_score_thresh = 3.5;
}