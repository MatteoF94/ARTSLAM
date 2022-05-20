#include "registration.h"

#include <iostream>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#ifdef USE_VGICP_CUDA
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

using namespace artslam::laser3d;

// Class constructor
Registration::Registration() {
    Configuration default_configuration;
    registration_method_ = select_registration_method(default_configuration);
}

// Class constructor, with parameters
Registration::Registration(const Configuration &configuration) {
    registration_method_ = select_registration_method(configuration);
}

// Returns the registration method
pcl::Registration<Point3I, Point3I>::Ptr Registration::registration_method() {
    return registration_method_;
}

// Changes the registration method
pcl::Registration<Point3I, Point3I>::Ptr Registration::change_registration_method(const Configuration& configuration) {
    registration_method_ = select_registration_method(configuration);
    return registration_method_;
}

// Creates the registration method, given configuration data
pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr Registration::select_registration_method(const Configuration& configuration) {
    std::stringstream msg;

    // select a registration method (ICP, GICP, NDT)
    if(configuration.registration_method_name_ == "FAST_GICP") {
        fast_gicp::FastGICP<Point3I, Point3I>::Ptr gicp(new fast_gicp::FastGICP<Point3I, Point3I>());
        gicp->setNumThreads(configuration.registration_num_threads_);
        gicp->setTransformationEpsilon(configuration.registration_transform_epsilon_);
        gicp->setMaximumIterations(configuration.registration_max_iterations_);
        gicp->setMaxCorrespondenceDistance(configuration.registration_max_corr_distance_);
        gicp->setCorrespondenceRandomness(configuration.registration_corr_randomness_);

        if(configuration.verbose_) {
            msg << "[Registration] Selected method: FAST_GICP\n";
            msg << "[Registration] \t- Registration num threads: " << configuration.registration_num_threads_ << "\n";
            msg << "[Registration] \t- Registration transformation epsilon: " << configuration.registration_transform_epsilon_ << "\n";
            msg << "[Registration] \t- Registration max iterations: " << configuration.registration_max_iterations_ << "\n";
            msg << "[Registration] \t- Registration max correspondence distance: " << configuration.registration_max_corr_distance_ << "\n";
            msg << "[Registration] \t- Registration correspondence randomness: " << configuration.registration_corr_randomness_ << "\n";
            std::cout << msg.str();
        }

        return gicp;
    }
    #ifdef USE_VGICP_CUDA
    else if(configuration.registration_method_name_ == "FAST_VGICP_CUDA") {
        fast_gicp::FastVGICPCuda<Point3I, Point3I>::Ptr vgicp(new fast_gicp::FastVGICPCuda<Point3I, Point3I>());
        vgicp->setResolution(configuration.registration_resolution_);
        vgicp->setTransformationEpsilon(configuration.registration_transform_epsilon_);
        vgicp->setMaximumIterations(configuration.registration_max_iterations_);
        vgicp->setCorrespondenceRandomness(configuration.registration_corr_randomness_);

        if(configuration.verbose_) {
            msg << "[Registration] Selected method: FAST_VGICP_CUDA\n";
            msg << "[Registration] \t- Registration resolution: " << configuration.registration_resolution_ << "\n";
            msg << "[Registration] \t- Registration transformation epsilon: " << configuration.registration_transform_epsilon_ << "\n";
            msg << "[Registration] \t- Registration max iterations: " << configuration.registration_max_iterations_ << "\n";
            msg << "[Registration] \t- Registration correspondence randomness: " << configuration.registration_corr_randomness_ << "\n";
            std::cout << msg.str();
        }

        return vgicp;
    }
    #endif
    else if(configuration.registration_method_name_ == "FAST_VGICP") {
        std::cout << "registration: FAST_VGICP" << std::endl;
        fast_gicp::FastVGICP<Point3I, Point3I>::Ptr vgicp(new fast_gicp::FastVGICP<Point3I, Point3I>());
        vgicp->setNumThreads(configuration.registration_num_threads_ );
        vgicp->setResolution(configuration.registration_resolution_);
        vgicp->setTransformationEpsilon(configuration.registration_transform_epsilon_);
        vgicp->setMaximumIterations(configuration.registration_max_iterations_);
        vgicp->setCorrespondenceRandomness(configuration.registration_corr_randomness_);

        if(configuration.verbose_) {
            msg << "[Registration] Selected method: FAST_VGICP\n";
            msg << "[Registration] \t- Registration num threads: " << configuration.registration_num_threads_ << "\n";
            msg << "[Registration] \t- Registration resolution: " << configuration.registration_resolution_ << "\n";
            msg << "[Registration] \t- Registration transformation epsilon: " << configuration.registration_transform_epsilon_ << "\n";
            msg << "[Registration] \t- Registration max iterations: " << configuration.registration_max_iterations_ << "\n";
            msg << "[Registration] \t- Registration correspondence randomness: " << configuration.registration_corr_randomness_ << "\n";
            std::cout << msg.str();
        }

        return vgicp;
    } else if(configuration.registration_method_name_ == "ICP") {
        std::cout << "registration: ICP" << std::endl;
        pcl::IterativeClosestPoint<Point3I, Point3I>::Ptr icp(new pcl::IterativeClosestPoint<Point3I, Point3I>());
        icp->setTransformationEpsilon(configuration.registration_transform_epsilon_);
        icp->setMaximumIterations(configuration.registration_max_iterations_);
        icp->setMaxCorrespondenceDistance(configuration.registration_max_corr_distance_);
        icp->setUseReciprocalCorrespondences(configuration.registration_use_reciprocal_corr_);

        if(configuration.verbose_) {
            msg << "[Registration] Selected method: ICP\n";
            msg << "[Registration] \t- Registration transformation epsilon: " << configuration.registration_transform_epsilon_ << "\n";
            msg << "[Registration] \t- Registration max iterations: " << configuration.registration_max_iterations_ << "\n";
            msg << "[Registration] \t- Registration max correspondence distance: " << configuration.registration_max_corr_distance_ << "\n";
            msg << "[Registration] \t- Registration use reciprocal correspondences: " << configuration.registration_use_reciprocal_corr_ << "\n";
            std::cout << msg.str();
        }

        return icp;
    } else if(configuration.registration_method_name_.find("GICP") != std::string::npos) {
        if(configuration.registration_method_name_.find("OMP") == std::string::npos) {
            pcl::GeneralizedIterativeClosestPoint<Point3I, Point3I>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<Point3I, Point3I>());
            gicp->setTransformationEpsilon(configuration.registration_transform_epsilon_);
            gicp->setMaximumIterations(configuration.registration_max_iterations_);
            gicp->setUseReciprocalCorrespondences(configuration.registration_use_reciprocal_corr_);
            gicp->setMaxCorrespondenceDistance(configuration.registration_max_corr_distance_ );
            gicp->setCorrespondenceRandomness(configuration.registration_corr_randomness_);
            gicp->setMaximumOptimizerIterations(configuration.registration_max_optimizer_iterations_);

            if(configuration.verbose_) {
                msg << "[Registration] Selected method: GICP\n";
                msg << "[Registration] \t- Registration transformation epsilon: " << configuration.registration_transform_epsilon_ << "\n";
                msg << "[Registration] \t- Registration max iterations: " << configuration.registration_max_iterations_ << "\n";
                msg << "[Registration] \t- Registration use reciprocal correspondences: " << configuration.registration_use_reciprocal_corr_ << "\n";
                msg << "[Registration] \t- Registration max correspondence distance: " << configuration.registration_max_corr_distance_ << "\n";
                msg << "[Registration] \t- Registration correspondence randomness: " << configuration.registration_corr_randomness_ << "\n";
                msg << "[Registration] \t- Registration max optimizer iterations: " << configuration.registration_max_optimizer_iterations_ << "\n";
                std::cout << msg.str();
            }

            return gicp;
        } else {
            pclomp::GeneralizedIterativeClosestPoint<Point3I, Point3I>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<Point3I, Point3I>());
            gicp->setTransformationEpsilon(configuration.registration_transform_epsilon_);
            gicp->setMaximumIterations(configuration.registration_max_iterations_);
            gicp->setUseReciprocalCorrespondences(configuration.registration_use_reciprocal_corr_);
            gicp->setMaxCorrespondenceDistance(configuration.registration_max_corr_distance_);
            gicp->setCorrespondenceRandomness(configuration.registration_corr_randomness_);
            gicp->setMaximumOptimizerIterations(configuration.registration_max_optimizer_iterations_);

            if(configuration.verbose_) {
                msg << "[Registration] Selected method: GICP_OMP\n";
                msg << "[Registration] \t- Registration transformation epsilon: " << configuration.registration_transform_epsilon_ << "\n";
                msg << "[Registration] \t- Registration max iterations: " << configuration.registration_max_iterations_ << "\n";
                msg << "[Registration] \t- Registration use reciprocal correspondences: " << configuration.registration_use_reciprocal_corr_ << "\n";
                msg << "[Registration] \t- Registration max correspondence distance: " << configuration.registration_max_corr_distance_ << "\n";
                msg << "[Registration] \t- Registration correspondence randomness: " << configuration.registration_corr_randomness_ << "\n";
                msg << "[Registration] \t- Registration max optimizer iterations: " << configuration.registration_max_optimizer_iterations_ << "\n";
                std::cout << msg.str();
            }

            return gicp;
        }
    } else {
        if(configuration.registration_method_name_.find("OMP") == std::string::npos) {
            pcl::NormalDistributionsTransform<Point3I, Point3I>::Ptr ndt(new pcl::NormalDistributionsTransform<Point3I, Point3I>());
            ndt->setTransformationEpsilon(configuration.registration_transform_epsilon_);
            ndt->setMaximumIterations(configuration.registration_max_iterations_);
            ndt->setResolution(configuration.ndt_resolution_);

            if(configuration.verbose_) {
                msg << "[Registration] Selected method: NDT\n";
                msg << "[Registration] \t- Registration transformation epsilon: " << configuration.registration_transform_epsilon_ << "\n";
                msg << "[Registration] \t- Registration max iterations: " << configuration.registration_max_iterations_ << "\n";
                msg << "[Registration] \t- Registration resolution: " << configuration.ndt_resolution_ << "\n";
                std::cout << msg.str();
            }

            return ndt;
        } else {
            pclomp::NormalDistributionsTransform<Point3I, Point3I>::Ptr ndt(new pclomp::NormalDistributionsTransform<Point3I, Point3I>());
            if(configuration.registration_num_threads_ > 0) {
                ndt->setNumThreads(configuration.registration_num_threads_);
            }
            ndt->setTransformationEpsilon(configuration.registration_transform_epsilon_);
            ndt->setMaximumIterations(configuration.registration_max_iterations_);
            ndt->setResolution(configuration.ndt_resolution_);
            if(configuration.registration_nn_search_method_ == "KDTREE") {
                ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
            } else if(configuration.registration_nn_search_method_ == "DIRECT1") {
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
            } else {
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
            }

            if(configuration.verbose_) {
                msg << "[Registration] Selected method: NDT_OMP\n";
                if(configuration.registration_num_threads_ > 0)
                    msg << "[Registration] \t- Registration num threads: " << configuration.registration_num_threads_ << "\n";
                msg << "[Registration] \t- Registration transformation epsilon: " << configuration.registration_transform_epsilon_ << "\n";
                msg << "[Registration] \t- Registration max iterations: " << configuration.registration_max_iterations_ << "\n";
                msg << "[Registration] \t- Registration resolution: " << configuration.ndt_resolution_ << "\n";
                if(configuration.registration_nn_search_method_ == "KDTREE") {
                    msg << "[Registration] \t- Registration nearest neighbour search method: KDTREE\n";
                } else if(configuration.registration_nn_search_method_ == "DIRECT1") {
                    msg << "[Registration] \t- Registration nearest neighbour search method: DIRECT1\n";
                } else {
                    msg << "[Registration] \t- Registration nearest neighbour search method: DIRECT7\n";
                }
                std::cout << msg.str();
            }

            return ndt;
        }
    }

    return nullptr;
}