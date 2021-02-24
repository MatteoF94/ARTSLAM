/** @file info_matrix_calculator.h
 * @brief Declaration of class InfoMatrixCalculator
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_INFO_MATRIX_CALCULATOR_H
#define ARTSLAM_INFO_MATRIX_CALCULATOR_H


#include <slam_types.h>
#include <info_matrix_calculator_config.h>
#include <limits>
#include <pcl/point_cloud.h>

/**
 * @class InfoMatrixCalculator
 * @brief This class is used to calculate the information matrix associated to the relative motion between two point clouds.
 */
class InfoMatrixCalculator {
public:
    /**
     * @brief Class constructor.
     */
    InfoMatrixCalculator();

    /**
     * @brief Class constructor, with parameter.
     * @param config_ptr Pointer to the configuration object for this class.
     */
    explicit InfoMatrixCalculator(const InfoMatrixCalculatorConfig *config_ptr);

    /**
     * @brief Class destructor.
     */
    ~InfoMatrixCalculator();

    // ---------------------------------------------------------------
    // ---------------- INFORMATION MATRIX COMPUTATION ---------------
    //----------------------------------------------------------------
    /**
     * @brief Calculates the information matrix associated to the relative motion between two point clouds
     * @param cloud_1 The first point cloud.
     * @param cloud_2 The second point cloud.
     * @param relative_pose The relative motion between the two point clouds.
     * @return The information matrix.
     */
    Eigen::MatrixXd calculate_info_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud_1, const pcl::PointCloud<PointT>::ConstPtr& cloud_2, const Eigen::Isometry3d& relative_pose) const;

    /**
     * @brief Calculates the information matrix associated to a pre-computed fitness score.
     * @param fitness_score Euclidean fitness score.
     * @return The information matrix.
     */
    Eigen::MatrixXd calculate_info_matrix(double fitness_score) const;

private:
    // ---------------------------------------------------------------
    // -------------------- CONFIGURATION METHODS --------------------
    //----------------------------------------------------------------
    /**
     * @brief Configures the tracker object using a configuration object.
     * @param config_ptr Pointer to the configuration object.
     */
    void configure_info_matrix_calculator(const InfoMatrixCalculatorConfig *config_ptr);

    // ---------------------------------------------------------------
    // ---------------- INFORMATION MATRIX COMPUTATION ---------------
    //----------------------------------------------------------------
    /**
     * @brief Calculates the fitness score, that is how much the motion aligns the point cloud.
     * @param cloud_1 The first point cloud.
     * @param cloud_2 The second point cloud.
     * @param relative_pose The relative motion between the two point clouds.
     * @param max_range Maximum acceptable fitness score.
     * @return The calculated fitness score.
     */
    static double calculate_fitness_score(const::pcl::PointCloud<PointT>::ConstPtr& cloud_1, const pcl::PointCloud<PointT>::ConstPtr& cloud_2, const Eigen::Isometry3d& relative_pose, double max_range = std::numeric_limits<double>::max());

    /**
     * @brief Computes the variance to be used when calculating the information matrix.
     * @param a Fitness gain, it tells how much the fitness matters.
     * @param max_x Maximum fitness score.
     * @param min_y Minimum variance.
     * @param max_y Maximum variance.
     * @param x Fitness score.
     * @return The computed variance.
     */
    static double weight(double a, double max_x, double min_y, double max_y, double x);

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


#endif //ARTSLAM_INFO_MATRIX_CALCULATOR_H
