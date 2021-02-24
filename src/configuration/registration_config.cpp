/** @file registration_config.cpp
 * @brief Definition of class RegistrationConfig
 */

#include <registration_config.h>

RegistrationConfig::RegistrationConfig() {
    this->registration_method = "NDT_OMP";
    this->reg_num_threads = 2;
    this->reg_transformation_epsilon = 0.01;
    this->reg_maximum_iterations = 64;
    this->reg_max_correspondence_distance = 2.5;
    this->reg_correspondence_randomness = 20;
    this->reg_resolution = 1.0;
    this->reg_use_reciprocal_correspondences = false;
    this->reg_max_optimizer_iterations = 20;
    this->reg_nn_search_method = "DIRECT7";
    this->ndt_resolution = 2;
}