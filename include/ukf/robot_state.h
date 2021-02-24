/** @file robot_state.h
 * @brief Declaration of multiple classes, representing the state of a moving robot: RobotState3D, RobotState2HALFD and
 * RobotState2D
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_ROBOT_STATE_H
#define ARTSLAM_ROBOT_STATE_H


#include <ukf_types.h>

/**
 * @class RobotState3D
 * @brief This class is used hold the state of a robot, used state of an Unscented Kalman Filter.
 * @details The position of the robot is expressed in the global 3D coordinate frames (easting, northing and elevation),
 * the rotation is expressed as a quaternion w.r.t. East, while the velocities follows the robot's frame: x is forward,
 * y is leftward and z is upward.
 */
class RobotState3D {
public:
    /**
     * @brief Class constructor.
     */
    RobotState3D();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    Vector10d state_vector;         /**< State: easting [m], northing [m], elevation [m], quat_w, quat_x, quat_y, quat_z, velocity x [m/s], velocity y [m/s], velocity z [m/s] */
    Matrix10d covariance_matrix;    /**< State covariance matrix */
};

/**
 * @class RobotState2HALFD
 * @brief This class is used hold the state of a robot, used state of an Unscented Kalman Filter.
 * @details The position of the robot is expressed in the global 3D coordinate frames (easting, northing and elevation),
 * the rotation is also expressed w.r.t. East, while the velocities follows the robot's frame: x is forward, y is leftward
 * and z is upward. The robot is assumed to not rotate around its forward and leftward axis.
 */
class RobotState2HALFD {
public:
    /**
     * @brief Class constructor.
     */
    RobotState2HALFD();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    Vector7d state_vector;          /**< State: easting [m], northing [m], elevation [m], heading [rad], velocity x [m/s], velocity y [m/s], velocity z [m/s] */
    Matrix7d covariance_matrix;     /**< State covariance matrix */
};


/**
 * @class RobotState2D
 * @brief This class is used hold the state of a robot, used state of an Unscented Kalman Filter.
 * @details The position of the robot is expressed in the global 2D coordinate frames (easting and northing), the rotation
 * is also expressed w.r.t. East, while the velocities follows the robot's frame: x is forward and y is leftward.
 */
class RobotState2D {
public:
    /**
     * @brief Class constructor.
     */
    RobotState2D();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    Vector5d state_vector;         /**< State: easting [m], northing [m], heading [rad], velocity x [m/s], velocity y [m/s] */
    Matrix5d covariance_matrix;    /**< State covariance matrix */
};

#endif //ARTSLAM_ROBOT_STATE_H
