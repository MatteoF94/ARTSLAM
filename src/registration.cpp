/** @file registration.cpp
 * @brief Definition of class Registration
 * @author Matteo Frosi
 */

#include <registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

/* ------------------ */
/* Class constructor. */
/* ------------------ */
Registration::Registration() {
    RegistrationConfig default_config;
    config_registration(&default_config);
}

/* ----------------------------------- */
/* Class constructor, with parameters. */
/* ----------------------------------- */
Registration::Registration(const RegistrationConfig* const config) {
    config_registration(config);
}


/* ---------------------------------------------------------------- */
/* Configures the registration object using a configuration object. */
/* ---------------------------------------------------------------- */
void Registration::config_registration(const RegistrationConfig* const config) {
    std::ostringstream to_print;

    to_print << "[Registration][configure_registration] creating and configurating the scan matching\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    this->registration_method = config->registration_method;

    this->reg_num_threads = config->reg_num_threads;
    this->reg_transformation_epsilon = config->reg_transformation_epsilon;
    this->reg_maximum_iterations = config->reg_maximum_iterations;
    this->reg_correspondence_randomness = config->reg_correspondence_randomness;
    this->reg_resolution = config->reg_resolution;
    this->reg_use_reciprocal_correspondences = config->reg_use_reciprocal_correspondences;
    this->reg_max_correspondence_distance = config->reg_max_correspondence_distance;
    this->reg_max_optimizer_iterations = config->reg_max_optimizer_iterations;
    this->reg_nn_search_method = config->reg_nn_search_method;
    this->ndt_resolution = config->ndt_resolution;

    to_print << "[Registration][configure_registration] REGISTRATION_METHOD: " << this->registration_method << "\n";
    to_print << "[Registration][configure_registration] REG_TRANSFORMATION_EPSILON: " << this->reg_transformation_epsilon << "\n";
    to_print << "[Registration][configure_registration] REG_MAXIMUM_ITERATIONS: " << this->reg_maximum_iterations << "\n";
    to_print << "[Registration][configure_registration] REG_CORRESPONDENCE_RANDOMNESS: " << this->reg_correspondence_randomness << "\n";
    to_print << "[Registration][configure_registration] REG_RESOLUTION: " << this->reg_resolution << "\n";
    to_print << "[Registration][configure_registration] REG_USE_RECIPROCAL_CORRESPONDENCES: " << this->reg_use_reciprocal_correspondences << "\n";
    to_print << "[Registration][configure_registration] REG_MAX_CORRESPONDENCE_DISTANCE: " << this->reg_max_correspondence_distance << "\n";
    to_print << "[Registration][configure_registration] REG_MAX_OPTIMIZER_ITERATIONS: " << this->reg_max_optimizer_iterations << "\n";
    to_print << "[Registration][configure_registration] REG_NN_SEARCH_METHOD: " << this->reg_nn_search_method << "\n";
    to_print << "[Registration][configure_registration] NDT_RESOLUTION: " << this->ndt_resolution << "\n";
    to_print << "[Registration][configure_registration] finished configurating the scan matching\n";
    std::cout << to_print.str();
}

/* ------------------------------------------------------------------------------ */
/* Builds and selects the registration method using the configuration parameters. */
/* ------------------------------------------------------------------------------ */
pcl::Registration<PointT,PointT>::Ptr Registration::select_registration_method() {
    if(this->registration_method == "FAST_GICP") {
        boost::shared_ptr<fast_gicp::FastGICP<PointT, PointT>> fgicp(new fast_gicp::FastGICP<PointT, PointT>());
        fgicp->setNumThreads(this->reg_num_threads);
        fgicp->setTransformationEpsilon(this->reg_transformation_epsilon);
        fgicp->setMaximumIterations(this->reg_maximum_iterations);
        fgicp->setMaxCorrespondenceDistance(this->reg_max_correspondence_distance);
        fgicp->setCorrespondenceRandomness(this->reg_correspondence_randomness);
        return fgicp;
    } else if(this->registration_method == "FAST_VGICP") {
        boost::shared_ptr<fast_gicp::FastVGICP<PointT, PointT>> fvgicp(new fast_gicp::FastVGICP<PointT, PointT>());
        fvgicp->setNumThreads(this->reg_num_threads);
        fvgicp->setResolution(this->reg_resolution);
        fvgicp->setTransformationEpsilon(this->reg_transformation_epsilon);
        fvgicp->setMaximumIterations(this->reg_maximum_iterations);
        fvgicp->setCorrespondenceRandomness(this->reg_correspondence_randomness);
        return fvgicp;
    } else if(this->registration_method == "ICP") {
        pcl::IterativeClosestPoint<PointT,PointT>::Ptr icp(new pcl::IterativeClosestPoint<PointT, PointT>());
        icp->setTransformationEpsilon(this->reg_transformation_epsilon);
        icp->setMaximumIterations(this->reg_maximum_iterations);
        icp->setMaxCorrespondenceDistance(this->reg_max_correspondence_distance);
        icp->setUseReciprocalCorrespondences(this->reg_use_reciprocal_correspondences);
        return icp;
    } else if(this->registration_method.find("GICP") != std::string::npos) {
        if(this->registration_method.find("OMP") == std::string::npos) {
            pcl::GeneralizedIterativeClosestPoint<PointT,PointT>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
            gicp->setTransformationEpsilon(this->reg_transformation_epsilon);
            gicp->setMaximumIterations(this->reg_maximum_iterations);
            gicp->setUseReciprocalCorrespondences(this->reg_use_reciprocal_correspondences);
            gicp->setMaxCorrespondenceDistance(this->reg_max_correspondence_distance);
            gicp->setCorrespondenceRandomness(this->reg_correspondence_randomness);
            gicp->setMaximumOptimizerIterations(this->reg_max_optimizer_iterations);
            return gicp;
        } else {
            pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
            gicp->setTransformationEpsilon(this->reg_transformation_epsilon);
            gicp->setMaximumIterations(this->reg_maximum_iterations);
            gicp->setUseReciprocalCorrespondences(this->reg_use_reciprocal_correspondences);
            gicp->setMaxCorrespondenceDistance(this->reg_max_correspondence_distance);
            gicp->setCorrespondenceRandomness(this->reg_correspondence_randomness);
            gicp->setMaximumOptimizerIterations(this->reg_max_optimizer_iterations);
            return gicp;
        }
    } else {
        if(this->registration_method.find("OMP") == std::string::npos) {
            pcl::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
            ndt->setTransformationEpsilon(this->reg_transformation_epsilon);
            ndt->setMaximumIterations(this->reg_maximum_iterations);
            ndt->setResolution(this->ndt_resolution);
            return ndt;
        } else {
            pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
            if(this->reg_num_threads > 0) {
                ndt->setNumThreads(this->reg_num_threads);
            }
            ndt->setTransformationEpsilon(this->reg_transformation_epsilon);
            ndt->setMaximumIterations(this->reg_maximum_iterations);
            ndt->setResolution(this->ndt_resolution);

            if(this->reg_nn_search_method == "KDTREE") {
                ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
            } else if(this->reg_nn_search_method == "DIRECT1") {
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
            } else {
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
            }
            return ndt;
        }
    }
}