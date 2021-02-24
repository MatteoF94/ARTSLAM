/** @file backend_config.h
 * @brief Declaration of class BackendConfig
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_BACKEND_CONFIG_H
#define ARTSLAM_BACKEND_CONFIG_H


#include <vector>
#include <slam_types.h>

/**
 * @class BackendConfig
 * @brief This class is used to store the configuration parameters to use in the Backend object.
 */
class BackendConfig {
public:
    /**
     * @brief Class constructor.
     */
    BackendConfig();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    int max_unopt_keyframes;                    /**< After how many keyframes loop closure is invoked */
    int optimizer_iters;                        /**< Number of iterations of the optimizer */

    // keyframes stuff
    bool use_anchor_node;                       /**< Whether or not the graph node of the first keyframe should be fixed */
    std::vector<double> anchor_node_stddev;     /**< Standard deviations of the anchor node */
    bool fix_first_node_adaptive;               /**< Whether or not the first node should be able to move freely around the origin */
    bool fix_first_keyframe_node;               /**< Whether or not the first node should be fixed (not the anchor) */
    std::string odometry_edge_robust_kernel;    /**< Name of the robust kernel for the keyframes edge */
    double odometry_edge_robust_kernel_size;    /**< Size of the robust kernel for the keyframes edge */

    // floor coeffs stuff
    double floor_edge_stddev;                   /**< Standard deviation to use for the pose graph information matrix for floor coefficients */
    std::string floor_edge_robust_kernel;       /**< Name of the robust kernel for the floor coefficients edge */
    double floor_edge_robust_kernel_size;       /**< Size of the robust kernel for the floor coefficients edge */

    // imu stuff
    bool is_imu_accel_enabled;                  /**< Whether or not IMU acceleration is used in the pose graph */
    bool is_imu_orien_enabled;                  /**< Whether or not IMU orientation is used in the pose graph */
    Eigen::Matrix3d imu_to_velo_rot;            /**< Rotation matrix from IMU to Velodyne */
    double imu_accel_edge_stddev;               /**< Standard deviation to use for the pose graph information matrix for acceleration */
    double imu_orien_edge_stddev;               /**< Standard deviation to use for the pose graph information matrix for orientation */
    std::string imu_accel_edge_robust_kernel;   /**< Name of the robust kernel for the acceleration edge */
    double imu_accel_edge_robust_kernel_size;   /**< Size of the robust kernel for the acceleration edge */
    std::string imu_orien_edge_robust_kernel;   /**< Name of the robust kernel for the orientation edge */
    double imu_orien_edge_robust_kernel_size;   /**< Size of the robust kernel for the orientation edge */

    // gps stuff
    bool is_gps_enabled;                        /**< Whether or nor GPS data is used in the pose graph */
    Eigen::Vector3d gps_to_velo_trans;          /**< Translation from GPS to Velodyne */
    double gps_xy_edge_stddev;                  /**< Standard deviation to use for the pose graph information matrix for planar coordinates */
    double gps_z_edge_stddev;                   /**< Standard deviation to use for the pose graph information matrix for 3D coordinates */
    std::string gps_edge_robust_kernel;         /**< Name of the robust kernel for the GPS edge */
    double gps_edge_robust_kernel_size;         /**< Size of the robust kernel for the GPS edge */

    // loop closure stuff
    std::string loop_edge_robust_kernel;        /**< Name of the robust kernel for the loop edge */
    double loop_edge_robust_kernel_size;        /**< Size of the robust kernel for the loop edge */
};


#endif //ARTSLAM_BACKEND_CONFIG_H
