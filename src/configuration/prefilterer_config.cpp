/** @file prefilterer_config.cpp
 * @brief Definition of class PrefiltererConfig
 * @author Matteo Frosi
*/

#include <prefilterer_config.h>

/* Class constructor. */
PrefiltererConfig::PrefiltererConfig() {
    this->downsample_method = "VOXELGRID";
    this->downsample_resolution = 0.1;
    this->sample_size = 30000;
    this->outlier_removal_method = "STATISTICAL";
    this->statistical_mean_k = 20;
    this->statistical_stddev = 1.0;
    this->radius_radius = 0.8;
    this->radius_min_neighbors = 2;
    this->use_distance_filter = true;
    this->distance_near_thresh = 1.0;
    this->distance_far_thresh = 100.0;
}