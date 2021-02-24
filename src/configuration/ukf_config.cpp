/** @file ukf_config.cpp
 * @brief Definition of class UKFConfig
 * @author Matteo Frosi
*/

#include <ukf_config.h>

/* Class constructor. */
UKFConfig::UKFConfig() {
    this->noise_var_pos = 1e-4;
    this->noise_var_quat = 1e-4;
    this->noise_var_vel = 1e-4;

    this->init_easting = 0.0;
    this->init_northing = 0.0;
    this->init_elevation = 0.0;
    this->init_quat_w = 0.0;
    this->init_quat_x = 0.0;
    this->init_quat_y = 0.0;
    this->init_quat_z = 0.0;
    this->init_forward_vel = 0.0;
    this->init_leftward_vel = 0.0;
    this->init_upward_vel = 0.0;
    this->init_var_pos = 1e-4;
    this->init_var_quat = 1e-4;
    this->init_var_vel = 1e-4;

    this->noise_var_pos_meas = 1e-3;
    this->noise_var_quat_meas = 1e-3;
    this->noise_var_vel_meas = 1e-3;

    this->control_var_ang_vel = 1e-3;
    this->control_var_accel = 1e-3;

    this->num_iterations = 10;

    this->alpha = 0.0;
    this->beta = 0.0;
    this->k = -5;
}