#ifndef ARTSLAM_LASER_3D_INFORMATION_MATRIX_CALCULATOR_H
#define ARTSLAM_LASER_3D_INFORMATION_MATRIX_CALCULATOR_H

#include <artslam_types/types_pcl.hpp>
#include <artslam_types/types_eigen.hpp>
#include <pcl/point_cloud.h>

using namespace artslam::core::types;

namespace artslam::laser3d {
    class InformationMatrixCalculator {
    public:
        struct Configuration {
            bool constant_information_matrix_ = false;
            double translation_constant_stddev_ = 0.5;
            double rotation_constant_stddev_ = 0.1;
            double variance_gain_ = 20.0;
            double minimum_translation_stddev_ = 0.1;
            double maximum_translation_stddev_ = 5.0;
            double minimum_rotation_stddev_ = 0.05;
            double maximum_rotation_stddev_ = 0.2;
            double fitness_score_threshold_ = 2.5;
            bool verbose_ = false;
        };

        InformationMatrixCalculator();

        explicit InformationMatrixCalculator(const Configuration& configuration);

        ~InformationMatrixCalculator();

        static double calc_fitness_score(const pcl::PointCloud<Point3I>::ConstPtr& cloud1, const pcl::PointCloud<Point3I>::ConstPtr& cloud2, const EigIsometry3d& relpose, double max_range = std::numeric_limits<double>::max());

        EigMatrixXd calc_information_matrix(const pcl::PointCloud<Point3I>::ConstPtr& cloud1, const pcl::PointCloud<Point3I>::ConstPtr& cloud2, const EigIsometry3d& relpose) const;

        EigMatrixXd compute_information_matrix(double fitness_score);

    private:
        double weight(double a, double max_x, double min_y, double max_y, double x) const;

    private:
        Configuration configuration_;
    };
}


#endif //ARTSLAM_LASER_3D_INFORMATION_MATRIX_CALCULATOR_H
