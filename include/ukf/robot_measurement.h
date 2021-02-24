/** @file robot_measurement.h
 * @brief Declaration of multiple classes, representing the measurement taken by one or multiple sensors:
 * RobotMeasurement3D, RobotMeasurement2HALFD and RobotMeasurement2D
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_ROBOT_MEASUREMENT_H
#define ARTSLAM_ROBOT_MEASUREMENT_H


#include <ukf_types.h>

/**
 * @class RobotMeasurement3D
 * @brief This class is used hold the measurements of one of multiple sensors of a robot, used to correct the state of
 * an Unscented Kalman Filter.
 * @details The position of the robot is expressed in the global 3D coordinate frames (easting, northing and elevation),
 * the rotation is expressed as a quaternion w.r.t. East, while the velocities follows the robot's frame: x is forward,
 * y is leftward and z is upward.
 */
class RobotMeasurement3D {
public:
    /**
     * @brief Class constructor.
     */
    RobotMeasurement3D();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    Vector10d measurement_vector;   /**< State: easting [m], northing [m], elevation [m], quat_w, quat_x, quat_y, quat_z, velocity x [m/s], velocity y [m/s], velocity z [m/s] */
    uint64_t timestamp;             /**< Measurement timestamp [ns] */
};

/**
 * @class RobotMeasurement2HALFD
 * @brief This class is used hold the measurements of one of multiple sensors of a robot, used to correct the state of
 * an Unscented Kalman Filter.
 * @details The position of the robot is expressed in the global 3D coordinate frames (easting, northing and elevation),
 * the rotation is also expressed w.r.t. East, while the velocities follows the robot's frame: x is forward, y is leftward
 * and z is upward. The robot is assumed to not rotate around its forward and leftward axis.
 */
class RobotMeasurement2HALFD {
public:
    /**
     * @brief Class constructor.
     */
    RobotMeasurement2HALFD();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    Vector7d measurement_vector;    /**< State: easting [m], northing [m], elevation [m], heading [rad], velocity x [m/s], velocity y [m/s], velocity z [m/s] */
    uint64_t timestamp;             /**< Measurement timestamp [ns] */
};

/**
 * @class RobotMeasurement2D
 * @brief This class is used hold the measurements of one of multiple sensors of a robot, used to correct the state of
 * an Unscented Kalman Filter.
 * @details The position of the robot is expressed in the global 2D coordinate frames (easting and northing), the rotation
 * is also expressed w.r.t. East, while the velocities follows the robot's frame: x is forward and y is leftward.
 */
class RobotMeasurement2D {
public:
    /**
     * @brief Class constructor.
     */
    RobotMeasurement2D();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    Vector5d measurement_vector;    /**< Measurement: easting [m], northing [m], heading [rad], velocity x [m/s], velocity y [m/s] */
    uint64_t timestamp;             /**< Measurement timestamp [ns] */
};


#endif //ARTSLAM_ROBOT_MEASUREMENT_H
