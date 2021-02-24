/** @file keyframe.h
 * @brief Declaration of class Keyframe
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_KEYFRAME_H
#define ARTSLAM_KEYFRAME_H


#include <slam_types.h>
#include <cstdint>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>
#include <g2o/types/slam3d/vertex_se3.h>

namespace g2o {
    class VertexSE3;
    class HyperGraph;
    class SparseOptimizer;
}

/**
 * @class Keyframe
 * @brief This class represents a keyframe.
 * @details A keyframe is a snapshot of the SLAM system under certain conditions, such as elapsed time, large angular
 * movement or large translation. It contains different variables of the system: timestamp, point cloud, floor coefficients,
 * GPS data, IMU data and the corresponding g2o node.
 */
class Keyframe {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief Smart pointer to a Keyframe object.
     */
    using Ptr = std::shared_ptr<Keyframe>;
    /**
     * @brief Smart pointer to a constant Keyframe object.
     */
    using ConstPtr = std::shared_ptr<const Keyframe>;

    /**
     * @brief Class constructor, with parameters.
     * @param timestamp Timestamp associated to the keyframe, expressed in nanoseconds.
     * @param odom Estimated odometry (by pose-tracking) of the data associated to the keyframe.
     * @param accum_distance Accumulated distance from the beginning.
     * @param cloud Point cloud associated to the keyframe
     */
    Keyframe(uint64_t timestamp, const Eigen::Isometry3d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud);

    /**
     * @brief Class constructor, with parameters.
     * @param directory Name of the directory where keyframe data is stored.
     * @param graph The current pose graph.
     */
    Keyframe(const std::string& directory, g2o::HyperGraph* graph);

    /**
     * @brief Class destructor.
     */
    virtual ~Keyframe();

    /**
     * @brief Saves the keyframe data in a directory.
     * @param directory The name of the directory where the keyframe should be saved.
     */
    void save_to_dir(const std::string& directory);

    /**
     * @brief Loads the keyframe from a directory.
     * @param directory The name of the directory where the keyframe should be loaded from.
     * @param graph The current pose graph.
     */
    void load_from_dir(const std::string& directory, g2o::HyperGraph* graph);

    /**
     * @brief Gets the keyframe ID (actually, the ID of the g2o node of the keyframe).
     * @return The keyframe ID.
     */
    long id() const;

    /**
     * @brief Gets the estimated and optimized pose of the keyframe.
     * @return The keyframe pose.
     */
    Eigen::Isometry3d  estimate() const;

public:
    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    uint64_t timestamp;                                 /**< Timestamp (in nanoseconds) of the keyframe */
    Eigen::Isometry3d odom;                             /**< Odometry estimated by the tracker */
    double accum_distance;                              /**< Accumulated distance from the beginning of the trajectory */
    double fitness_score;                               /**< Euclidean fitness of the matching between this keyframe and the previous one */
    pcl::PointCloud<PointT>::ConstPtr cloud;            /**< Point cloud associated to the keyframe */
    boost::optional<Eigen::Vector4d> floor_coeffs;      /**< Floor coefficients associated to the keyframe */
    boost::optional<Eigen::Vector3d> utm_coords;        /**< UTM coordinates associated to the keyframe */
    boost::optional<Eigen::Vector3d> acceleration;      /**< Linear acceleration associated to the keyframe */
    boost::optional<Eigen::Quaterniond> orientation;    /**< Orientation associated to the keyframe */
    g2o::VertexSE3* node;                               /**< g2o node containing the optimized pose of the keyframe */
};


#endif //ARTSLAM_KEYFRAME_H
