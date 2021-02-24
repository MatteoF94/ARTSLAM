/** @file loop_detector_config.h
 * @brief Declaration of class LoopDetectorConfig
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_LOOP_DETECTOR_CONFIG_H
#define ARTSLAM_LOOP_DETECTOR_CONFIG_H


/**
 * @class LoopDetectorConfig
 * @brief This class is used to store the configuration parameters to use in the LoopDetectorConfig object.
 * @author Matteo Frosi
 */
class LoopDetectorConfig {
public:
    /**
     * @brief Class constructor.
     */
    LoopDetectorConfig();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    double distance_thresh;                 /**< Maximum estimated distance between keyframes to be considered candidates for a loop */
    double accum_distance_thresh;           /**< Minimum travelled distance between keyframes to be considered candidates for a loop */
    double distance_from_last_edge_thresh;  /**< Minimum distance between the new and the last loop edges */

    double fitness_score_max_range;         /**< Maximum allowable distance between corresponding points */
    double fitness_score_thresh;            /**< Threshold used for scan matching */
};


#endif //ARTSLAM_LOOP_DETECTOR_CONFIG_H
