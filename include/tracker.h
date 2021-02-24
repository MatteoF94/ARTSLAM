/** @file tracker.h
 * @brief Declaration of class Tracker
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_TRACKER_H
#define ARTSLAM_TRACKER_H


#include <slam_types.h>
#include <filtered_cloud_observer.h>
#include <odom_observer.h>
#include <odom_prior_observer.h>
#include <keyframe_observer.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <dispatch_queue.h>
#include <tracker_config.h>
#include <registration.h>
#include <keyframe_handler.h>
#include <keyframe.h>

/**
 * @class Tracker
 * @brief This class is used to track the motions of the robot between consecutive time steps.
 */
class Tracker : public FilteredCloudObserver, public OdomPriorObserver {
public:
    /**
     * @brief Class constructor.
     */
    Tracker();

    /**
     * @brief Class constructor, with parameter.
     * @param config_ptr Pointer to the configuration object for this class.
     */
    explicit Tracker(const TrackerConfig *config_ptr);

    /**
     * @brief Class destructor.
     */
    ~Tracker();

    /**
     * @brief Sets the registration method used for scan matching.
     * @param registration Pointer to the registration method.
     */
    void set_registration(Registration* registration);

    // ---------------------------------------------------------------
    // --------------------- IO OBSERVER METHODS ---------------------
    //----------------------------------------------------------------
    /**
     * @brief Registers an object waiting for odometries.
     * @param odom_observer_ptr_tmp Pointer to the observer object.
     */
    void register_odom_observer(OdomObserver *odom_observer_ptr_tmp);

    /**
     * @brief Removes an object waiting for odometries.
     * @param odom_observer_ptr_tmp Pointer to the observer object.
     */
    void remove_odom_observer(OdomObserver *odom_observer_ptr_tmp);

    /**
     * @brief Notifies all the observers with the estimated odometry.
     * @param header Header associated to the estimated odometry.
     * @param odom Estimated odometry.
     */
    void notify_odom_observers(const Header& header, const Eigen::Matrix4f& odom);

    /**
     * @brief Registers an object waiting for keyframes.
     * @param kf_observer_ptr_tmp Pointer to the observer object.
     */
    void register_keyframe_observer(KeyframeObserver *kf_observer_ptr_tmp);

    /**
     * @brief Removes an object waiting for keyframes.
     * @param kf_observer_ptr_tmp Pointer to the observer object.
     */
    void remove_keyframe_observer(KeyframeObserver *kf_observer_ptr_tmp);

    /**
     * @brief Notifies all the observers with the computed keyframe.
     * @param keyframe The keyframe to send to the observers.
     */
    void notify_keyframe_observers(const Keyframe::Ptr& keyframe);

    /**
     * @brief Signals that a new filtered point cloud has been received.
     * @param src_cloud The filtered point cloud received and to be used for pose tracking.
     */
    void update(pcl::PointCloud<PointT>::ConstPtr src_cloud) override;

    /**
     * @brief Signals that a prior odometry has been received.
     * @param header General information about the received odometry.
     * @param odom The received odometry.
     */
    void update(const Header& header, const Eigen::Matrix4f& odom) override;

private:
    // ---------------------------------------------------------------
    // -------------------- CONFIGURATION METHODS --------------------
    //----------------------------------------------------------------
    /**
     * @brief Configures the tracker object using a configuration object.
     * @param config_ptr Pointer to the configuration object.
     */
    void configure_tracker(const TrackerConfig *config_ptr);

    /**
     * @brief Configures the downsampling method used to filter point clouds.
     * @param config_ptr Pointer to the configuration object.
     */
    void configure_downsample(const TrackerConfig *config_ptr);

    // ---------------------------------------------------------------
    // ----------------------- TRACKING METHODS ----------------------
    //----------------------------------------------------------------
    /**
     * @brief Computes the tracking between the current point cloud and the previous keyframe.
     * @param cloud The current point cloud.
     */
    void compute_tracking(const pcl::PointCloud<PointT>::ConstPtr& cloud);

    /**
     * @brief Finds the relative motion between point clouds.
     * @param timestamp The timestamp (nanoseconds) of the current point cloud.
     * @param cloud The current point cloud.
     * @return The motion of the robot expressed in the classic rototranslation matrix
     */
    Eigen::Matrix4f matching(uint64_t timestamp, const pcl::PointCloud<PointT>::ConstPtr& cloud);

    /**
     * @brief Downsamples a point cloud.
     * @param cloud The point cloud to downsample.
     * @return The downsampled point cloud.
     */
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;

    /**
     * @brief Updates the current odometry, searching for a new keyframe.
     * @param odom The current estimated odometry.
     * @param timestamp The timestamp corresponding at the current odometry.
     * @param seq_id The sequence number of the current point cloud / frame.
     * @return Whether or not a new keyframe is created.
     */
    bool update_odometry(const Eigen::Matrix4f& odom, uint64_t timestamp, int seq_id);

    /**
     * @brief Computes the motion guess to use when scan matching using the available pre-computed odometries.
     * @param timestamp Timestamp of the current frame.
     * @return The guessed rototranslation.
     */
    Eigen::Matrix4f compute_prior_odom_guess(uint64_t timestamp);

    /**
     * @brief Finds the pre-computed odometry closest in time to the current frame (not keyframe).
     * @param timestamp Timestamp of the current frame.
     * @return The closest odometry, identity if none.
     */
    Eigen::Matrix4f find_closest_prior_odom(uint64_t timestamp);

    // ---------------------------------------------------------------
    // ------------------------- OTHER METHODS -----------------------
    //----------------------------------------------------------------
    /**
     * @brief Creates a new keyframe.
     * @return The created keyframe.
     */
    Keyframe::Ptr create_keyframe();

    /**
    * @brief Tells the tracker that a point cloud is ready to be dispatched for tracking.
    * @param cloud The point cloud to be used for pose tracking.
    */
    void dispatch_tracking(const pcl::PointCloud<PointT>::ConstPtr& cloud);

    /**
     * @brief Tells the tracker that prior odometry is ready to be dispatched for insertion.
     * @param header General information about the received odometry.
     * @param odom The received odometry.
     */
    void dispatch_prior_odom_insertion(const Header& header, const Eigen::Matrix4f& odom);

    /**
     * @brief Adds a pre-computed odometry, to be used when performing scan matching as initial guess.
     * @param header General information about the received odometry.
     * @param odom The received odometry.
     */
    void add_prior_odom(const Header& header, const Eigen::Matrix4f& odom);

    /**
     * @brief Saves the prefiltering times to file, for benchmarking
     * @param filename Name of the file where to dump the prefiltering times
     */
    void dump_times(const std::string& filename);

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    double keyframe_delta_trans;    /**< Minimum distance between keyframes */
    double keyframe_delta_angle;    /**< Minimum rotation between keyframes */
    double keyframe_delta_time;     /**< Minimum timestamp difference between keyframes */

    bool transform_thresholding;    /**< Whether or not the obtained transformation should be thresholded */
    double max_acceptable_trans;    /**< Maximum acceptable distance between frames */
    double max_acceptable_angle;    /**< Maximum acceptable angle between frames */

    bool is_first_frame;            /**< Whether or not the frame considered is the first */
    double accumulated_distance;    /**< Accumulated distance of the keyframes */

    bool skip_mode;                     /**< Whether or not the traker is in frame skipping mode */
    bool skip_by_ukf;                   /**< Whether or not the tracker should be into skip mode, using UKF prior odometries */

    Eigen::Matrix4f prev_trans;         /**< Previous transform from the keyframe */
    Eigen::Matrix4f keyframe_pose;      /**< Pose of the keyframe */
    double current_fitness;             /**< Euclidean fitness score of the current scan matching */
    uint64_t keyframe_stamp;            /**< Keyframe timestamp (nanoseconds) */
    pcl::PointCloud<PointT>::ConstPtr keyframe_cloud;       /**< Point cloud of the active keyframe */
    pcl::PointCloud<PointT>::ConstPtr curr_filtered_cloud;  /**< Current filtered point cloud */

    pcl::Filter<PointT>::Ptr downsample_filter;                 /**< Filter used to downsample the point clouds */
    pcl::Registration<PointT,PointT>::Ptr registration_method;  /**< Method used to perform scan matching between point clouds */

    DispatchQueue* tracker_dqueue;                      /**< Queue used to dispatch multiple trackings in order */
    std::vector<OdomObserver*> odom_observers;          /**< Vector of objects waiting for odometries */
    std::vector<KeyframeObserver*> keyframe_observers;  /**< Vector of objects waiting for keyframes */

    Registration* registration;         /**< Used to select the registration method */

    std::mutex prior_odom_mutex;                                    /**< Mutex used when handling the prior odometries */
    std::deque<std::pair<uint64_t, Eigen::Matrix4f>> prior_odoms;   /**< Contains timestamps and associated pre-computed odometries */
    Eigen::Matrix4f keyframe_prior_odom;                            /**< Pre-computed odometry associated to the current keyframe */
    Eigen::Matrix4f current_prior_odom;                             /**< Pre-computed odometry associated to the current point cloud */
    bool has_current_prior_odom;                                    /**< Whether or not the current frame has a corresponding pre-computed odometry */
    double prior_odom_delta_trans;                                  /**< Minimum distance between pre-computed odometries to guess a new keyframe */
    double prior_odom_delta_angle;                                  /**< Minimum rotation between pre-computed odometries to guess a new keyframe */
    DispatchQueue* prior_odom_insertion_dqueue;                     /**< Queue used to dispatch multiple prior odometries insertions */

    std::vector<uint64_t> times;    /**< Contains all the times spent while prefiltering */
    uint64_t total_time = 0;        /**< Total execution time of the prefilterer */

    std::vector<Eigen::Matrix4f> odoms; /**< Stores all the odometries computed, can be used to perform super offline SLAM */
};


#endif //ARTSLAM_TRACKER_H
