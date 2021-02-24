/** @file info_matrix_calculator_config.h
 * @brief Declaration of class InfoMatrixCalculatorConfig
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_INFO_MATRIX_CALCULATOR_CONFIG_H
#define ARTSLAM_INFO_MATRIX_CALCULATOR_CONFIG_H


/**
 * @class InfoMatrixCalculatorConfig
 * @brief This class is used to store the configuration parameters to use in the InfoMatrixCalculator object.
 * @author Matteo Frosi
 */
class InfoMatrixCalculatorConfig {
public:
    /**
     * @brief Class constructor.
     */
    InfoMatrixCalculatorConfig();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    bool use_const_info_matrix;     /**< Whether or not use a constant information matrix */
    double const_stddev_x;          /**< Constant standard deviation for the translational motion */
    double const_stddev_q;          /**< Constant standard deviation for the rotational motion */

    double var_gain_a;              /**< Fitness variance gain, it tells how much the fitness matters when computing the variance */
    double min_stddev_x;            /**< Minimum standard deviation for the translational motion */
    double max_stddev_x;            /**< Maximum standard deviation for the translational motion */
    double min_stddev_q;            /**< Minimum standard deviation for the rotational motion */
    double max_stddev_q;            /**< Maximum standard deviation for the rotational motion */
    double fitness_score_thresh;    /**< Maximum fitness score */
};


#endif //ARTSLAM_INFO_MATRIX_CALCULATOR_CONFIG_H
