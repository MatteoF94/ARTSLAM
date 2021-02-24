/** @file registration.h
 * @brief Declaration of class Registration
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_REGISTRATION_H
#define ARTSLAM_REGISTRATION_H


#include <slam_types.h>
#include <pcl/registration/registration.h>
#include <registration_config.h>

/**
 * @class Registration
 * @brief This class is used perform scan matching using the PCL library.
 */
class Registration {
public:
    /**
     * @brief Class constructor.
     */
    Registration();

    /**
     * @brief Class constructor, with parameters.
     * @param config Pointer to the configuration object for this class.
     */
    explicit Registration(const RegistrationConfig* config);

    /**
     * @brief Builds and selects the registration method using the configuration parameters.
     * @return The registration method selected for scan matching.
     */
    pcl::Registration<PointT, PointT>::Ptr select_registration_method();

private:
    // ---------------------------------------------------------------
    // -------------------- CONFIGURATION METHODS --------------------
    //----------------------------------------------------------------
    /**
     * @brief Configures the registration object using a configuration object.
     * @param config_ptr Pointer to the configuration object.
     */
    void config_registration(const RegistrationConfig* config_ptr);

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    std::string registration_method;            /**< Registration method: FAST GICP, FAST VGICP, ICP, GICP, GICP OMP, NDT, NDT OMP */
    int reg_num_threads;                        /**< FAST GICP, FAST VGICP, NDT OMPS: number of threads used */
    double reg_transformation_epsilon;          /**< FAST GICP, FAST VGICP, ICP, GICP, NDT, NDT OMP: distance termination criterion */
    int reg_maximum_iterations;                 /**< FAST GICP, FAST VGICP, ICP, GICP, NDT, NDT OMP: iterations termination criterion */
    double reg_max_correspondence_distance;     /**< FAST GICP, FAST VGICP, ICP, GICP: distance for which correspondences in clouds are ignored */
    int reg_correspondence_randomness;          /**< FAST GICP, FAST VGICP, GICP, GICP OMP: number of neighbors used for covariance computation */
    double reg_resolution;                      /**< FAST VGICP: voxel resolution */
    bool reg_use_reciprocal_correspondences;    /**< ICP, GICP, GICP OMP: whether reciprocal correspondences are used or not */
    int reg_max_optimizer_iterations;           /**< GICP, GICP OMP: maximum number of iterations at the optimization step */
    std::string reg_nn_search_method;           /**< NDT, NDT OMP: neighborood search method (KDTREE, DIRECT1, DIRECT7) */
    float ndt_resolution;                       /**< NDT, NDT OMP: resolution */
};


#endif //ARTSLAM_REGISTRATION_H
