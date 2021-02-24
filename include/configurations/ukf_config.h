/** @file ukf_config.h
 * @brief Declaration of class UKFConfig
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_UKF_CONFIG_H
#define ARTSLAM_UKF_CONFIG_H


#include <stdint.h>

/**
 * @class UKFConfig
 * @brief This class is used to store the configuration parameters to use in the UKF objects.
 */
class UKFConfig {
public:
    /**
     * @brief Class constructor.
     */
    UKFConfig();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    double noise_var_pos;       /**< Noise variance for the position of the robot */
    double noise_var_quat;      /**< Noise variance for the orientation of the robot */
    double noise_var_vel;       /**< Noise variance for the velocity of the robot */

    double init_easting;        /**< Initial easting */
    double init_northing;       /**< Initial northing */
    double init_elevation;      /**< Initial elevation */
    double init_quat_w;         /**< Initial quaternion parameter w */
    double init_quat_x;         /**< Initial quaternion parameter x */
    double init_quat_y;         /**< Initial quaternion parameter y */
    double init_quat_z;         /**< Initial quaternion parameter z */
    double init_forward_vel;    /**< Initial forward velocity (w.r.t. the robot) */
    double init_leftward_vel;   /**< Initial leftward velocity (w.r.t. the robot) */
    double init_upward_vel;     /**< Initial upward velocity (w.r.t. the robot) */
    double init_var_pos;        /**< Initial variance for the position of the robot */
    double init_var_quat;       /**< Initial variance for the orientation of the robot */
    double init_var_vel;        /**< Initial variance for the velocity of the robot */

    double noise_var_pos_meas;  /**< Noise variance for the measured position of the robot */
    double noise_var_quat_meas; /**< Noise variance for the measured orientation of the robot */
    double noise_var_vel_meas;  /**< Noise variance for the measured velocity of the robot */

    double control_var_ang_vel; /**< Variance for the angular velocity used as control input */
    double control_var_accel;   /**< Variance for the acceleration used as control input */

    uint16_t num_iterations;    /**< Number of partial iteration at each prediction step of the UKF */

    double alpha;               /**< Parameter alpha of the UKF */
    double beta;                /**< Parameter beta of the UKF */
    uint8_t k;                  /**< Parameter k of the UKF */
};


#endif //ARTSLAM_UKF_CONFIG_H
