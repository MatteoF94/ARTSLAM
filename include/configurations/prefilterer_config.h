/** @file prefilterer_config.h
 * @brief Declaration of class PrefiltererConfig
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_PREFILTERER_CONFIG_H
#define ARTSLAM_PREFILTERER_CONFIG_H


#include <string>

/**
 * @class PrefiltererConfig
 * @brief This class is used to store the configuration parameters to use in the Prefilterer object.
 */
class PrefiltererConfig {
public:
    /**
     * @brief Class constructor.
     */
    PrefiltererConfig();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    std::string downsample_method;          /**< Downsample method, can be VOXELGRID, APPROX_VOXELGRID, UNIFORM, RANDOM or none */
    double downsample_resolution;           /**< Resolution of the leaf/radius used during the downsampling */
    int sample_size;                        /**< Desired size of the downsampled cloud, for RANDOM downsampling */

    std::string outlier_removal_method;     /**< Outlier removal method, can be STATISTICAL, RADIUS or none */
    int statistical_mean_k;                 /**< Mean param for the STATISTICAL outlier removal method */
    double statistical_stddev;              /**< Standard deviation param for the STATISTICAL outlier removal method */
    double radius_radius;                   /**< Radius param for the RADIUS outlier removal method */
    int radius_min_neighbors;               /**< Minimum number of neighbors for the RADIUS outlier removal method */

    bool use_distance_filter;               /**< Toggles whether a point cloud is filtered by the range of its points */
    double distance_near_thresh;            /**< Minimum point range[m] from the sensor, for the distance filter */
    double distance_far_thresh;             /**< Maximum point range[m] from the sensor, for the distance filter */
};


#endif //ARTSLAM_PREFILTERER_CONFIG_H
