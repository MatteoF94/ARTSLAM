/** @file robot_control_input.h
 * @brief Declaration of class IMUControlInput
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_ROBOT_CONTROL_INPUT_H
#define ARTSLAM_ROBOT_CONTROL_INPUT_H


#include <slam_types.h>

/**
 * @class IMUControlInput
 * @brief This class is used hold the measurements of an IMU, used as control input for an Unscented Kalman Filter.
 */
class IMUControlInput {
public:
    /**
     * @brief Class constructor.
     */
    IMUControlInput();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    double gyro_x;          /**< Angular velocity w.r.t. the x axis */
    double gyro_y;          /**< Angular velocity w.r.t. the y axis */
    double gyro_z;          /**< Angular velocity w.r.t. the z axis */
    double accel_x;         /**< Acceleration w.r.t. the x axis */
    double accel_y;         /**< Acceleration w.r.t. the y axis */
    double accel_z;         /**< Acceleration w.r.t. the z axis */
    uint64_t timestamp;     /**< Timestamp (in nanoseconds) of the control input */
};

#endif //ARTSLAM_ROBOT_CONTROL_INPUT_H
