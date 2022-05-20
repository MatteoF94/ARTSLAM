#include "information_matrix_calculator.h"
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

using namespace artslam::laser3d;

InformationMatrixCalculator::InformationMatrixCalculator() {
    std::stringstream msg;

    if(configuration_.verbose_) {
        msg << "[InformationMatrixCalculator] Creating and configuring the information matrix calculator\n";
        std::cout << msg.str();
        msg.str("");
    }

    if(configuration_.verbose_) {
        msg << "[InformationMatrixCalculator] Finished creating and configuring the information matrix calculator\n";
        std::cout << msg.str();
    }
}

InformationMatrixCalculator::InformationMatrixCalculator(const Configuration &configuration) {
    std::stringstream msg;

    configuration_.verbose_ = configuration.verbose_;
    if(configuration_.verbose_) {
        msg << "[InformationMatrixCalculator] Creating and configuring the information matrix calculator\n";
        std::cout << msg.str();
        msg.str("");
    }

    configuration_.constant_information_matrix_ = configuration.constant_information_matrix_;
    configuration_.translation_constant_stddev_ = configuration.translation_constant_stddev_;
    configuration_.rotation_constant_stddev_ = configuration.rotation_constant_stddev_;
    configuration_.variance_gain_ = configuration.variance_gain_;
    configuration_.minimum_translation_stddev_ = configuration.minimum_translation_stddev_;
    configuration_.maximum_translation_stddev_ = configuration.maximum_translation_stddev_;
    configuration_.minimum_rotation_stddev_ = configuration.minimum_rotation_stddev_;
    configuration_.maximum_rotation_stddev_ = configuration.maximum_rotation_stddev_;
    configuration_.fitness_score_threshold_ = configuration.fitness_score_threshold_;

    if(configuration_.verbose_) {
        msg << "[InformationMatrixCalculator] Finished creating and configuring the information matrix calculator\n";
        std::cout << msg.str();
    }
}

InformationMatrixCalculator::~InformationMatrixCalculator() = default;

EigMatrixXd InformationMatrixCalculator::calc_information_matrix(const pcl::PointCloud<Point3I>::ConstPtr& cloud1, const pcl::PointCloud<Point3I>::ConstPtr& cloud2, const EigIsometry3d& relpose) const {
    if(configuration_.constant_information_matrix_) {
        Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
        inf.topLeftCorner(3, 3).array() /= configuration_.translation_constant_stddev_;
        inf.bottomRightCorner(3, 3).array() /= configuration_.rotation_constant_stddev_;
        return inf;
    }

    double fitness_score = calc_fitness_score(cloud1, cloud2, relpose);

    double min_var_x = std::pow(configuration_.minimum_translation_stddev_, 2);
    double max_var_x = std::pow(configuration_.maximum_translation_stddev_, 2);
    double min_var_q = std::pow(configuration_.minimum_rotation_stddev_, 2);
    double max_var_q = std::pow(configuration_.maximum_rotation_stddev_, 2);

    float w_x = weight(configuration_.variance_gain_, configuration_.fitness_score_threshold_, min_var_x, max_var_x, fitness_score);
    float w_q = weight(configuration_.variance_gain_, configuration_.fitness_score_threshold_, min_var_q, max_var_q, fitness_score);

    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    inf.topLeftCorner(3, 3).array() /= w_x;
    inf.bottomRightCorner(3, 3).array() /= w_q;
    return inf;
}

double InformationMatrixCalculator::calc_fitness_score(const pcl::PointCloud<Point3I>::ConstPtr& cloud1, const pcl::PointCloud<Point3I>::ConstPtr& cloud2, const EigIsometry3d& relpose, double max_range) {
    pcl::search::KdTree<Point3I>::Ptr tree_(new pcl::search::KdTree<Point3I>());
    tree_->setInputCloud(cloud1);

    double fitness_score = 0.0;

    // Transform the input dataset using the final transformation
    pcl::PointCloud<Point3I> input_transformed;
    pcl::transformPointCloud(*cloud2, input_transformed, relpose.cast<float>());

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    // For each point in the source dataset
    int nr = 0;
    for(size_t i = 0; i < input_transformed.points.size(); ++i) {
        // Find its nearest neighbor in the target
        tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);

        // Deal with occlusions (incomplete targets)
        if(nn_dists[0] <= max_range) {
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if(nr > 0)
        return (fitness_score / nr);
    else
        return (std::numeric_limits<double>::max());
}

EigMatrixXd InformationMatrixCalculator::compute_information_matrix(double fitness_score) {
    if(configuration_.constant_information_matrix_) {
        EigMatrixXd information_matrix = EigMatrixXd::Identity(6,6);
        information_matrix.topLeftCorner(3,3).array() /= configuration_.translation_constant_stddev_ * configuration_.translation_constant_stddev_;
        information_matrix.bottomRightCorner(3,3).array() /= configuration_.rotation_constant_stddev_ * configuration_.rotation_constant_stddev_;
        return information_matrix;
    }

    double minimum_translation_variance = configuration_.minimum_translation_stddev_ * configuration_.minimum_translation_stddev_;
    double maximum_translation_variance = configuration_.maximum_translation_stddev_ * configuration_.maximum_translation_stddev_;
    double minimum_rotation_variance = configuration_.minimum_rotation_stddev_ * configuration_.minimum_rotation_stddev_;
    double maximum_rotation_variance = configuration_.maximum_rotation_stddev_ * configuration_.maximum_rotation_stddev_;

    double translation_weight = weight(configuration_.variance_gain_, configuration_.fitness_score_threshold_, minimum_translation_variance, maximum_translation_variance, fitness_score);
    double rotation_weight = weight(configuration_.variance_gain_, configuration_.fitness_score_threshold_, minimum_rotation_variance, maximum_rotation_variance, fitness_score);

    EigMatrixXd information_matrix = EigMatrixXd::Identity(6,6);
    information_matrix.topLeftCorner(3,3).array() /= translation_weight;
    information_matrix.bottomRightCorner(3,3).array() /= rotation_weight;
    return information_matrix;
}

double InformationMatrixCalculator::weight(double a, double max_x, double min_y, double max_y, double x) const {
    double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
    return min_y + (max_y - min_y) * y;
}