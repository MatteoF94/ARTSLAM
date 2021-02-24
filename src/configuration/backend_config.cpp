//
// Created by matteo on 19/02/21.
//

#include <backend_config.h>

BackendConfig::BackendConfig() {
    this->max_unopt_keyframes = 10;
    this->optimizer_iters = 512;

    // keyframes stuff
    this->use_anchor_node = true;
    this->anchor_node_stddev = {10,10,10,1,1,1};
    this->fix_first_node_adaptive = true;
    this->fix_first_keyframe_node = false;
    this->odometry_edge_robust_kernel = "NONE";
    this->odometry_edge_robust_kernel_size = 1.0;

    // floor coeffs stuff
    this->floor_edge_stddev = 1.0;
    this->floor_edge_robust_kernel = "NONE";
    this->floor_edge_robust_kernel_size = 1.0;

    // imu stuff
    this->is_imu_accel_enabled = false;
    this->is_imu_orien_enabled = false;
    this->imu_to_velo_rot = Eigen::Matrix3d::Identity();
    this->imu_accel_edge_stddev = 1.0;
    this->imu_orien_edge_stddev = 1.0;
    this->imu_accel_edge_robust_kernel = "NONE";
    this->imu_accel_edge_robust_kernel_size = 1.0;
    this->imu_orien_edge_robust_kernel = "NONE";
    this->imu_orien_edge_robust_kernel_size = 1.0;

    // gps stuff
    this->is_gps_enabled = false;
    this->gps_to_velo_trans = Eigen::Vector3d::Zero();
    this->gps_xy_edge_stddev = 20.0;
    this->gps_z_edge_stddev = 5.0;
    this->gps_edge_robust_kernel = "NONE";
    this->gps_edge_robust_kernel_size = 1.0;

    // loop closure stuff
    this->loop_edge_robust_kernel = "Huber";
    this->loop_edge_robust_kernel_size = 1.0;
}
