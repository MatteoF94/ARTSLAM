/** @file floor_detector_config.h
 * @brief Declaration of class FloorDetectorConfig
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_FLOOR_DETECTOR_CONFIG_H
#define ARTSLAM_FLOOR_DETECTOR_CONFIG_H


/**
 * @class FloorDetectorConfig
 * @brief This class is used to store the configuration parameters to use in the FloorDetector object.
 */
class FloorDetectorConfig {
public:
    /**
     * @brief Class constructor.
     */
    FloorDetectorConfig();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    double tilt_deg;                /**< Sensor tilt in degrees */
    double sensor_height;           /**< Sensor height */
    double height_clip_range;       /**< Range in which the floor can be searched (above and below sensor height) */

    int floor_pts_thresh;           /**< Minimum number of cloud points to be searched for floors */
    double floor_normal_thresh;     /**< Acceptable tilt of the normal of the detected floor */

    bool use_normal_filtering;      /**< Whether to use normal filtering or not */
    double normal_filter_thresh;    /**< Acceptable tilt of the normal of the detected floor, used for filtering */
};


#endif //ARTSLAM_FLOOR_DETECTOR_CONFIG_H
