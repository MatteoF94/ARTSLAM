/** @file info_matrix_calculator.cpp
 * @brief Definition of class InfoMatrixCalculator
 * @author Matteo Frosi
 */

#include <info_matrix_calculator.h>
#include <cmath>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

/* ------------------ */
/* Class constructor. */
/* ------------------ */
InfoMatrixCalculator::InfoMatrixCalculator() {
    InfoMatrixCalculatorConfig default_config;
    configure_info_matrix_calculator(&default_config);
}

/* ---------------------------------- */
/* Class constructor, with parameter. */
/* ---------------------------------- */
InfoMatrixCalculator::InfoMatrixCalculator(const InfoMatrixCalculatorConfig *config_ptr) {
    configure_info_matrix_calculator(config_ptr);
}

/* ----------------- */
/* Class destructor. */
/* ----------------- */
InfoMatrixCalculator::~InfoMatrixCalculator() = default;

/* ----------------------------------------------------------- */
/* Configures the tracker object using a configuration object. */
/* ----------------------------------------------------------- */
void InfoMatrixCalculator::configure_info_matrix_calculator(const InfoMatrixCalculatorConfig *config_ptr) {
    std::ostringstream to_print;

    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] creating and configurating the information matrix calculator\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    this->use_const_info_matrix = config_ptr->use_const_info_matrix;
    this->const_stddev_x = config_ptr->const_stddev_x;
    this->const_stddev_q = config_ptr->const_stddev_q;

    this->var_gain_a = config_ptr->var_gain_a;
    this->min_stddev_x = config_ptr->min_stddev_x;
    this->max_stddev_x = config_ptr->max_stddev_x;
    this->min_stddev_q = config_ptr->min_stddev_q;
    this->max_stddev_q = config_ptr->max_stddev_q;
    this->fitness_score_thresh = config_ptr->fitness_score_thresh;

    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] USE_CONST_INFO_MATRIX: " << this->use_const_info_matrix << "\n";
    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] CONST_STDDEV_X: " << this->const_stddev_x << "\n";
    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] CONST_STDDEV_Q: " << this->const_stddev_q << "\n";
    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] VAR_GAIN_A: " << this->var_gain_a << "\n";
    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] MIN_STDDEV_X: " << this->min_stddev_x << "\n";
    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] MAX_STDDEV_X: " << this->max_stddev_x << "\n";
    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] MIN_STDDEV_Q: " << this->min_stddev_q << "\n";
    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] MAX_STDDEV_Q: " << this->max_stddev_q << "\n";
    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] FITNESS_SCORE_THRESH: " << this->fitness_score_thresh << "\n";
    to_print << "[InfoMatrixCalculator][configure_info_matrix_calculator] finished configurating the information matrix calculator\n";
    std::cout << to_print.str();
}

/* -------------------------------------------------------------------------------------------- */
/* Calculates the information matrix associated to the relative motion between two point clouds */
/* -------------------------------------------------------------------------------------------- */
Eigen::MatrixXd InfoMatrixCalculator::calculate_info_matrix(const pcl::PointCloud<PointT>::ConstPtr &cloud_1,
                                                            const pcl::PointCloud<PointT>::ConstPtr &cloud_2,
                                                            const Eigen::Isometry3d &relative_pose) const {
    if(this->use_const_info_matrix) {
        Eigen::MatrixXd info_matrix = Eigen::MatrixXd::Identity(6,6);
        info_matrix.topLeftCorner(3,3).array() /= std::pow(this->const_stddev_x,2);
        info_matrix.bottomRightCorner(3,3).array() /= std::pow(this->const_stddev_q,2);
        return info_matrix;
    }

    double fitness_score = calculate_fitness_score(cloud_1, cloud_2, relative_pose);

    double min_var_x = std::pow(this->min_stddev_x,2);
    double max_var_x = std::pow(this->max_stddev_x,2);
    double min_var_q = std::pow(this->min_stddev_q, 2);
    double max_var_q = std::pow(this->max_stddev_q, 2);

    double w_x = weight(this->var_gain_a, this->fitness_score_thresh, min_var_x, max_var_x, fitness_score);
    double w_q = weight(this->var_gain_a, this->fitness_score_thresh, min_var_q, max_var_q, fitness_score);

    Eigen::MatrixXd info_matrix = Eigen::MatrixXd::Identity(6,6);
    info_matrix.topLeftCorner(3,3).array() /= w_x;
    info_matrix.bottomRightCorner(3,3).array() /= w_q;
    return info_matrix;
}

/* ----------------------------------------------------------------------------- */
/* Calculates the information matrix associated to a pre-computed fitness score. */
/* ----------------------------------------------------------------------------- */
Eigen::MatrixXd InfoMatrixCalculator::calculate_info_matrix(double fitness_score) const {
    if(this->use_const_info_matrix) {
        Eigen::MatrixXd info_matrix = Eigen::MatrixXd::Identity(6,6);
        info_matrix.topLeftCorner(3,3).array() /= std::pow(this->const_stddev_x,2);
        info_matrix.bottomRightCorner(3,3).array() /= std::pow(this->const_stddev_q,2);
        return info_matrix;
    }

    double min_var_x = std::pow(this->min_stddev_x,2);
    double max_var_x = std::pow(this->max_stddev_x,2);
    double min_var_q = std::pow(this->min_stddev_q, 2);
    double max_var_q = std::pow(this->max_stddev_q, 2);

    double w_x = weight(this->var_gain_a, this->fitness_score_thresh, min_var_x, max_var_x, fitness_score);
    double w_q = weight(this->var_gain_a, this->fitness_score_thresh, min_var_q, max_var_q, fitness_score);

    Eigen::MatrixXd info_matrix = Eigen::MatrixXd::Identity(6,6);
    info_matrix.topLeftCorner(3,3).array() /= w_x;
    info_matrix.bottomRightCorner(3,3).array() /= w_q;
    return info_matrix;
}

/* --------------------------------------------------------------------------------- */
/* Calculates the fitness score, that is how much the motion aligns the point cloud. */
/* --------------------------------------------------------------------------------- */
double InfoMatrixCalculator::calculate_fitness_score(const ::pcl::PointCloud<PointT>::ConstPtr &cloud_1,
                                                     const pcl::PointCloud<PointT>::ConstPtr &cloud_2,
                                                     const Eigen::Isometry3d &relative_pose, double max_range) {
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
    kdtree->setInputCloud(cloud_1);

    double fitness_score = 0.0;

    // transform using the final transformation
    pcl::PointCloud<PointT> input_transformed;
    pcl::transformPointCloud(*cloud_2, input_transformed, relative_pose.cast<float>());

    std::vector<int> nn_indices(1);
    std::vector<float> nn_distances(1); // they are actually SQUARED distances

    int nr = 0;
    for(auto & point : input_transformed.points) {
        kdtree->nearestKSearch(point, 1, nn_indices, nn_distances);

        if(nn_distances[0] < max_range) {
            fitness_score += nn_distances[0];
            nr++;
        }
    }

    if(nr > 0) {
        return (fitness_score / nr);
    } else {
        return (std::numeric_limits<double>::max());
    }
}

/* ------------------------------------------------------------------------- */
/* Computes the variance to be used when calculating the information matrix. */
/* ------------------------------------------------------------------------- */
double InfoMatrixCalculator::weight(double a, double max_x, double min_y, double max_y, double x) {
    double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
    return min_y + (max_y - min_y) * y;
}