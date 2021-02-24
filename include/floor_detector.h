/** @file floor_detector.h
* @brief Declaration of class FloorDetector
*/

#ifndef ARTSLAM_FLOOR_DETECTOR_H
#define ARTSLAM_FLOOR_DETECTOR_H


#include <slam_types.h>
#include <filtered_cloud_observer.h>
#include <floor_coeffs_observer.h>
#include <pcl/point_cloud.h>
#include <floor_detector_config.h>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>
#include <dispatch_queue.h>

/**
 * @class FloorDetector
 * @brief This class is used to detect floors in a point cloud.
 * @author Matteo Frosi
 */
class FloorDetector : public FilteredCloudObserver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Class constructor.
     */
    FloorDetector();

    /**
     * @brief Class constructor, with parameters.
     * @param config_ptr Pointer to the configuration object for this class.
     */
    explicit FloorDetector(const FloorDetectorConfig *config_ptr);

    /**
     * @brief Class destructor.
     */
    ~FloorDetector();

    // ---------------------------------------------------------------
    // --------------------- IO OBSERVER METHODS ---------------------
    //----------------------------------------------------------------
    /**
     * @brief Registers an object waiting for floor coefficients.
     * @param fc_observer_ptr_tmp Pointer to the observer object.
     */
    void register_floor_coeffs_observer(FloorCoeffsObserver *fc_observer_ptr_tmp);

    /**
     * @brief Removes an object waiting for floor coefficients.
     * @param fc_observer_ptr_tmp Pointer to the observer object.
     */
    void remove_floor_coeffs_observer(FloorCoeffsObserver *fc_observer_ptr_tmp);

    /**
     * @brief Notifies all the observers with estimated floor coefficients.
     */
    void notify_floor_coeffs_observers(const FloorCoeffsMSG::ConstPtr& floor_coeffs_msg);

    /**
     * @brief It signals that a new point cloud has been received.
     * @param src_cloud The point cloud received and used for floor detection.
     */
    void update(pcl::PointCloud<PointT>::ConstPtr src_cloud) override;

private:
    // ---------------------------------------------------------------
    // -------------------- CONFIGURATION METHODS --------------------
    //----------------------------------------------------------------
    /**
     * @brief Configures the floor detector object using a configuration object.
     * @param config_ptr Pointer to the configuration object.
     */
    void configure_floor_detector(const FloorDetectorConfig *const config_ptr);

    // ---------------------------------------------------------------
    // ------------------- FLOOR DETECTION METHODS -------------------
    //----------------------------------------------------------------
    /**
     * @brief It detects a floor in a point cloud.
     * @param cloud The point cloud where to search the floor in.
     */
    void floor_detection(const pcl::PointCloud<PointT>::ConstPtr& cloud);

    /**
     * @brief Performs the main operations needed to detect a floor.
     * @param cloud The point cloud where to search the floor in.
     * @return The normal of the detected floor.
     */
    boost::optional<Eigen::Vector4d> detect(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;

    /**
     * @brief Clips the point cloud to reduce the range search of the floor.
     * @param src_cloud The point cloud where to search the floor in.
     * @param plane Normal used for clipping
     * @param negative Whether to cut above or below the clipping plane.
     * @return The clipped point cloud.
     */
    pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative) const;

    /**
     * @brief Filters the point cloud depending on the normals of its points, after having computed them.
     * @param cloud The point cloud to filter.
     * @return The filtered point cloud.
     */
    pcl::PointCloud<PointT>::Ptr normal_filtering(const pcl::PointCloud<PointT>::Ptr& cloud) const;

    // ---------------------------------------------------------------
    // ------------------------- OTHER METHODS -----------------------
    //----------------------------------------------------------------
    /**
     * @brief Tells the floor detector that a point cloud is ready to be dispatched for floor detection.
    * @param cloud The point cloud where to search the floor in.
     */
    void dispatch_floor_detection(const pcl::PointCloud<PointT>::ConstPtr& cloud);

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    double tilt_deg;                        /**< Sensor tilt in degrees */
    double sensor_height;                   /**< Sensor height */
    double height_clip_range;               /**< Range in which the floor can be searched (above and below sensor height) */

    int floor_pts_thresh;                   /**< Minimum number of cloud points to be searched for floors */
    double floor_normal_thresh;             /**< Acceptable tilt of the normal of the detected floor */

    bool use_normal_filtering;              /**< Whether to use normal filtering or not */
    double normal_filter_thresh;            /**< Acceptable tilt of the normal of the detected floor, used for filtering */

    DispatchQueue* floor_detector_queue;    /**< Queue used to dispatch multiple floor detections in order */

    std::vector<FloorCoeffsObserver*> floor_coeffs_observers;   /**< Vector of objects waiting for floor coefficients */
};


#endif //ARTSLAM_FLOOR_DETECTOR_H
