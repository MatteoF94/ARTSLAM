/** @file prefilterer.h
 * @brief Declaration of class Prefilterer
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_PREFILTERER_H
#define ARTSLAM_PREFILTERER_H


#include <slam_types.h>
#include <cloud_observer.h>
#include <imu_observer.h>
#include <prefilterer_config.h>
#include <dispatch_queue.h>
#include <tracker.h>
#include <floor_detector.h>
#include <vector>
#include <pcl/filters/filter.h>

/**
 * @class Prefilterer
 * @brief This class is used to filter one or multiple point clouds.
 */
class Prefilterer : public CloudObserver, public ImuObserver {
public:
    /**
     * @brief Class constructor.
     */
    Prefilterer();

    /**
     * @brief Class constructor, with parameter.
     * @param config_ptr Pointer to the configuration object for this class.
     */
    explicit Prefilterer(const PrefiltererConfig *config_ptr);

    /**
     * @brief Class destructor.
     */
    ~Prefilterer();

    // ---------------------------------------------------------------
    // --------------------- IO OBSERVER METHODS ---------------------
    //----------------------------------------------------------------
    /**
     * @brief Registers an object waiting for filtered clouds.
     * @param fc_observer_ptr_tmp Pointer to the observer object.
     */
    void register_filtered_cloud_observer(FilteredCloudObserver *fc_observer_ptr_tmp);

    /**
     * @brief Removes an object waiting for filtered clouds.
     * @param fc_observer_ptr_tmp Pointer to the observer object.
     */
    void remove_filtered_cloud_observer(FilteredCloudObserver *fc_observer_ptr_tmp);

    /**
     * @brief Notifies all the observers with a new filtered cloud.
     */
    void notify_filtered_cloud_observers(const pcl::PointCloud<PointT>::ConstPtr& filtered_cloud);

    /**
     * @brief Signals that a new raw point cloud has been received.
     * @param src_cloud The raw point cloud received and to be filtered.
     */
    void update(pcl::PointCloud<PointT>::ConstPtr src_cloud) override;

    /**
     * @brief Signals that new IMU data has been received.
     * @param imu_msg_constptr The IMU data used to update the concrete object.
     */
    void update(const ImuMSG::ConstPtr& imu_msg_constptr) override;

private:
    // ---------------------------------------------------------------
    // -------------------- CONFIGURATION METHODS --------------------
    //----------------------------------------------------------------
    /**
     * @brief Configures the prefilterer object using a configuration object.
     * @param config_ptr Pointer to the configuration object.
     */
    void configure_prefilterer(const PrefiltererConfig *config_ptr);

    /**
     * @brief Configures the downsampling method used to filter point clouds.
     * @param config_ptr Pointer to the configuration object.
     */
    void configure_downsample(const PrefiltererConfig *config_ptr);

    /**
     * @brief Configures the outlier removal method used to filter point clouds.
     * @param config_ptr Pointer to the configuration object.
     */
    void configure_outlier_removal(const PrefiltererConfig *config_ptr);

    // ---------------------------------------------------------------
    // ---------------------- FILTERING METHODS ----------------------
    //----------------------------------------------------------------
    /**
     * @brief Filters and adjusts a point cloud.
     * @param cloud The point cloud to filter and adjust.
     */
    void filter_cloud(pcl::PointCloud<PointT>::ConstPtr cloud);

    /**
     * @brief Deskews a point cloud using IMU information.
     * @param cloud The point cloud to deskew.
     * @return The deskewed point cloud.
     */
    pcl::PointCloud<PointT>::ConstPtr deskewing(const pcl::PointCloud<PointT>::ConstPtr& cloud);

    /**
     * @brief Filters a point cloud using the range of its points.
     * @param cloud The point cloud to filter.
     * @return The filtered point cloud.
     */
    pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;

    /**
     * @brief Downsamples a point cloud.
     * @param cloud The point cloud to downsample.
     * @return The downsampled point cloud.
     */
    pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;

    /**
     * @brief Removes the outliers from a point cloud.
     * @param cloud The point cloud to which remove the outliers from.
     * @return The point cloud cleaned from its outliers.
     */
    pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;

    // ---------------------------------------------------------------
    // ------------------------- OTHER METHODS -----------------------
    //----------------------------------------------------------------
    /**
     * @brief Tells the prefilterer that a point cloud is ready to be dispatched for filtering.
     * @param cloud Pointer to the point cloud to filter.
    */
    void dispatch_filter_cloud(pcl::PointCloud<PointT>::ConstPtr& cloud);

    /**
     * @brief Tells the prefilterer that new IMU data is ready to be dispatched for insertion.
     * @param imu_msg_constptr Pointer to the IMU data to insert.
     */
    void dispatch_imu_insertion(const ImuMSG::ConstPtr& imu_msg_constptr);

    /**
     * @brief Adds an IMU measurement, to be used when deskewing a point cloud.
     * @param imu_msg The IMU measurement.
     */
    void add_imu_measurement(const ImuMSG::ConstPtr& imu_msg_constptr);

    /**
     * @brief Saves the prefiltering times to file, for benchmarking
     * @param filename Name of the file where to dump the prefiltering times
     */
    void dump_times(const std::string& filename);

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    bool use_distance_filter;       /**< Toggles whether a point cloud is filtered by the range of its points */
    double distance_near_thresh;    /**< Minimum point range[m] from the sensor, for the distance filter */
    double distance_far_thresh;     /**< Maximum point range[m] from the sensor, for the distance filter */

    pcl::Filter<PointT>::Ptr downsample_filter;         /**< Pointer to the filter used to downsample point clouds */
    std::vector<pcl::Filter<PointT>::Ptr> or_filters;   /**< Pointers to the filters used to remove outliers from the point clouds */

    std::string base_link_frame;                 /**< Name of the base link */
    Eigen::Matrix4f pc_to_base_link_transform;   /**< Transformation matrix from laser sensor to base link */
    double scan_period;                          /**< Scan period of the laser sensor */

    DispatchQueue* prefilterer_dqueue;           /**< Queue used to dispatch multiple point cloud filterings in order */
    DispatchQueue* imu_insertion_dqueue;         /**< Queue used to dispatch the insertion of new IMU data */
    std::vector<ImuMSG::ConstPtr> imu_queue;     /**< Queue of IMU measurements used for deskewing */
    std::mutex imu_insertion_mutex;              /**< Mutex used when inserting into or handling the IMU queue */

    std::vector<FilteredCloudObserver*> filtered_cloud_observers;   /**< Vector of objects waiting for filtered point clouds */

    std::vector<uint64_t> times;    /**< Contains all the times spent while prefiltering */
    uint64_t total_time = 0;        /**< Total execution time of the prefilterer */
};


#endif //ARTSLAM_PREFILTERER_H
