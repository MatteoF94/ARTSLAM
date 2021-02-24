/** @file tracker_config.h
 * @brief Declaration of class TrackerConfig
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_TRACKER_CONFIG_H
#define ARTSLAM_TRACKER_CONFIG_H


#include <string>

/**
 * @class TrackerConfig
 * @brief This class is used to store the configuration parameters to use in the Tracker object.
 */
class TrackerConfig {
public:
    /**
     * @brief Class constructor.
     */
    TrackerConfig();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    std::string downsample_method;      /**< Downsample method, can be VOXELGRID, APPROX_VOXELGRID  or none */
    float downsample_resolution;        /**< Resolution of the leaf used during the downsampling */

    double keyframe_delta_trans;        /**< Minimum distance between keyframes */
    double keyframe_delta_angle;        /**< Minimum rotation between keyframes */
    double keyframe_delta_time;         /**< Minimum timestamp difference between keyframes */

    bool transform_thresholding;        /**< Whether or not the obtained transformation should be thresholded */
    double max_acceptable_trans;        /**< Maximum acceptable distance between frames */
    double max_acceptable_angle;        /**< Maximum acceptable angle between frames */

    unsigned short num_frames_to_skip;  /**< Number of frames to skip */

    double prior_odom_delta_trans;      /**< Minimum distance between pre-computed odometries to guess a new keyframe */
    double prior_odom_delta_angle;      /**< Minimum rotation between pre-computed odometries to guess a new keyframe */
};


#endif //ARTSLAM_TRACKER_CONFIG_H
