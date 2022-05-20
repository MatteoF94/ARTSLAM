#ifndef ARTSLAM_LASER_3D_REGISTRATION_H
#define ARTSLAM_LASER_3D_REGISTRATION_H

#include <pcl/registration/registration.h>
#include <artslam_types/types_pcl.hpp>

using namespace artslam::core::types;

namespace artslam::laser3d {
    class Registration {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Defines the configuration parameters of the class
        struct Configuration {
            std::string registration_method_name_ = "FAST_GICP";    // registration method name
            int registration_num_threads_ = 0;                      // number of threads used
            double registration_transform_epsilon_ = 0.01;          // distance termination criterion
            int registration_max_iterations_ = 64;                  // iterations termination criterion
            double registration_max_corr_distance_ = 2.5;           // allowed distance between correspondences of two point clouds
            int registration_corr_randomness_ = 10;                 // best N matches to search for a correspondence
            double registration_resolution_ = 1.0;                  // voxel resolution for voxel-based registration
            bool registration_use_reciprocal_corr_ = false;         // whether reciprocal correspondences are used
            int registration_max_optimizer_iterations_ = 20;        // maximum number of iterations in the optimization step
            std::string registration_nn_search_method_ = "DIRECT7"; // neighbourhood search method
            float ndt_resolution_ = 0.5;                            // NDT registration resolution
            bool verbose_ = true;                                   // whether the class should be verbose
        };

        // Class constructor
        Registration();

        // Class constructor, with parameters
        explicit Registration(const Configuration& configuration);

        // Changes the registration method
        pcl::Registration<Point3I, Point3I>::Ptr change_registration_method(const Configuration& configuration);

        // Returns the registration method
        pcl::Registration<Point3I, Point3I>::Ptr registration_method();

    private:
        // Creates the registration method, given configuration data
        pcl::Registration<Point3I, Point3I>::Ptr select_registration_method(const Configuration& configuration);

        // ----------------------------------------------------------------------------------
        // ---------------------------- PARAMETERS AND VARIABLES ----------------------------
        // ----------------------------------------------------------------------------------
        pcl::Registration<Point3I, Point3I>::Ptr registration_method_;  // registration method
    };
}


#endif //ARTSLAM_LASER_3D_REGISTRATION_H
