/** @file info_matrix_calculator_config.cpp
 * @brief Definition of class InfoMatrixCalculatorConfig
 * @author Matteo Frosi
 */

#include <info_matrix_calculator_config.h>

/* ------------------ */
/* Class constructor. */
/* ------------------ */
InfoMatrixCalculatorConfig::InfoMatrixCalculatorConfig() {
    this->use_const_info_matrix = false;
    this->const_stddev_x = 0.5;
    this->const_stddev_q = 0.1;

    this->var_gain_a = 20.0;
    this->min_stddev_x = 0.1;
    this->max_stddev_x = 5.0;
    this->min_stddev_q = 0.05;
    this->max_stddev_q = 0.2;
    this->fitness_score_thresh = 0.5;
}