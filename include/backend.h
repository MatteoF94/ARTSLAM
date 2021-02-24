/** @file backend.h
 * @brief Declaration of class Backend
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_BACKEND_H
#define ARTSLAM_BACKEND_H


#include <string>
#include <mutex>
#include <deque>
#include <keyframe.h>
#include <dispatch_queue.h>
#include <graph_handler.h>
#include <loop_detector.h>
#include <keyframe_observer.h>
#include <floor_coeffs_observer.h>
#include <imu_observer.h>
#include <gps_observer.h>
#include <info_matrix_calculator.h>
#include <backend_config.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

class Backend : public KeyframeObserver, public FloorCoeffsObserver, public ImuObserver, public GpsObserver {
public:
    /**
     * @brief Class constructor.
     */
    Backend();

    /**
     * @brief Class constructor, with parameter.
     * @param config_ptr Pointer to the configuration object for this class.
     */
    explicit Backend(const BackendConfig* config_ptr);

    /**
     * @brief Class destructor.
     */
    ~Backend();

    /**
     * @brief Sets the pose graph handler.
     * @param graph_handler_ptr Pointer to the pose graph handler.
     */
    void set_graph_handler(GraphHandler* graph_handler_ptr);

    /**
     * @brief Sets the information matrix calculator.
     * @param info_matrix_calculator_ptr Pointer to the information matrix calculator.
     */
    void set_info_matrix_calculator(InfoMatrixCalculator* info_matrix_calculator_ptr);

    /**
     * @brief Sets the loop detector.
     * @param loop_detector_ptr Pointer to the loop detector.
     */
    void set_loop_detector(LoopDetector* loop_detector_ptr);

    /**
     * @brief Signals that a new keyframe has arrived.
     * @param keyframe Pointer to the waited keyframe.
     */
    void update(const Keyframe::Ptr& keyframe) override;

    /**
     * @brief Signals that a new set of floor coefficients has arrived.
     * @param floor_coeffs_msg_constptr Pointer to the waited set of floor coefficients.
     */
    void update(const FloorCoeffsMSG::ConstPtr& floor_coeffs_msg_constptr) override;

    /**
     * @brief Signals that new IMU data has arrived.
     * @param imu_msg_constptr Pointer to the IMU data.
     */
    void update(const ImuMSG::ConstPtr& imu_msg_constptr) override;

    /**
     * @brief Signals that new GPS data has arrived.
     * @param gps_msg_constptr Pointer to the GPS data.
     */
    void update(const GeoPointStampedMSG::ConstPtr& gps_msg_constptr) override;


    void save_data(const std::string& path);
    void optimize();

private:

    void configure_backend(const BackendConfig *config_ptr);

    void loop_detection();

    // ---------------------------------------------------------------
    // ----------------------- FLUSHING METHODS ----------------------
    //----------------------------------------------------------------
    /**
     * @brief Adds keyframes poses to the pose graph.
     */
    void flush_keyframe_queue();

    /**
     * @brief Associates floor coefficients to the keyframes, adding constraints in the pose graph.
     */
    void flush_floor_coeffs_msg_queue();

    /**
     * @brief Associates IMU data to the keyframes, adding constraints in the pose graph.
     */
    void flush_imu_msg_queue();

    /**
     * @brief Associates GPS data to the keyframes, adding constraints in the pose graph.
     */
    void flush_gps_msg_queue();

    // ---------------------------------------------------------------
    // ----------------------- INSERTION METHODS ---------------------
    //----------------------------------------------------------------
    /**
     * @brief Adds a keyframe to the backend.
     * @param keyframe Pointer to the keyframe.
     */
    void insert_keyframe(const Keyframe::Ptr& keyframe);

    /**
     * @brief Adds computed floor coefficients to the backend.
     * @param floor_coeffs_msg_constptr Pointer to the floor coefficients.
     */
    void insert_floor_coeffs_msg(const FloorCoeffsMSG::ConstPtr& floor_coeffs_msg_constptr);

    /**
     * @brief Adds IMU data to the backend.
     * @param imu_msg_constptr Pointer to the IMU data.
     */
    void insert_imu_msg(const ImuMSG::ConstPtr& imu_msg_constptr);

    /**
     * @brief Adds GPS data to the backend.
     * @param gps_msg_constptr Pointer to the GPS data.
     */
    void insert_gps_msg(const GeoPointStampedMSG::ConstPtr& gps_msg_constptr);

    // ---------------------------------------------------------------
    // ------------------------- OTHER METHODS -----------------------
    //----------------------------------------------------------------
    /**
    * @brief Tells the backend that a keyframe has arrived and it is waiting to be integrated in the pose graph.
    * @param keyframe Pointer to the keyframe to be inserted.
    */
    void dispatch_keyframe_insertion(const Keyframe::Ptr& keyframe);

    /**
     * @brief Tells the backend that a set of floor coefficients has arrived and it is waiting to be integrated in the pose graph.
     * @param floor_coeffs_msg_constptr Pointer to the set of floor coefficients to be inserted.
     */
    void dispatch_floor_coeffs_msg_insertion(const FloorCoeffsMSG::ConstPtr& floor_coeffs_msg_constptr);

    /**
     * @brief Tells the backend that new IMU data has arrived and it is waiting to be integrated in the pose graph.
     * @param imu_msg_constptr Pointer to the IMU data to be inserted.
     */
    void dispatch_imu_msg_insertion(const ImuMSG::ConstPtr& imu_msg_constptr);

    /**
     * @brief Tells the backend that new GPS data has arrived and it is waiting to be integrated in the pose graph.
     * @param gps_msg_constptr Pointer to the GPS data to be inserted.
     */
    void dispatch_gps_msg_insertion(const GeoPointStampedMSG::ConstPtr& gps_msg_constptr);

    /**
     * @brief Converts a rotation matrix to its quaternion form.
     * @param rot Rotation matrix.
     * @return The quaternion form of the input rotation matrix.
     */
    static Eigen::Quaterniond rot_to_quat(const Eigen::Matrix3d& rot);

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    int max_unopt_keyframes;                    /**< After how many keyframes loop closure is invoked */
    int optimizer_iters;                        /**< Number of iterations of the optimizer */

    // mutexes
    std::mutex backend_thread_mutex;            /**< Mutex for the loop detection and optimization thread */
    std::mutex trans_odom2map_mutex;            /**< Mutex for the transformation from odometry to map */
    std::mutex keyframe_queue_mutex;            /**< Mutex for the keyframe queue */
    std::mutex floor_coeffs_msg_queue_mutex;    /**< Mutex for the floor coefficients queue */
    std::mutex imu_msg_queue_mutex;             /**< Mutex for the IMU data queue */
    std::mutex gps_msg_queue_mutex;             /**< Mutex for the GPS data queue */

    // queues
    std::deque<Keyframe::Ptr> new_keyframes;                        /**< Keyframes to be tested for a loop closure */
    std::deque<Keyframe::Ptr> keyframe_queue;                       /**< Incoming keyframes queue */
    std::deque<FloorCoeffsMSG::ConstPtr> floor_coeffs_msg_queue;    /**< Floor coefficients queue */
    std::deque<ImuMSG::ConstPtr> imu_msg_queue;                     /**< IMU data queue */
    std::deque<GeoPointStampedMSG::ConstPtr> gps_msg_queue;         /**< IMU data queue */

    // keyframes stuff
    bool use_anchor_node;                       /**< Whether or not the graph node of the first keyframe should be fixed */
    std::vector<double> anchor_node_stddev;     /**< Standard deviations of the anchor node */
    g2o::VertexSE3* anchor_node;                /**< Fixed anchor node, connected to the first keyframe pose */
    g2o::EdgeSE3* anchor_edge;                  /**< Edge in the pose graph between the anchor node and first pose */
    bool fix_first_node_adaptive;               /**< Whether or not the first node should be able to move freely around the origin */
    bool fix_first_keyframe_node;               /**< Whether or not the first node should be fixed (not the anchor) */
    std::string odometry_edge_robust_kernel;    /**< Name of the robust kernel for the keyframes edge */
    double odometry_edge_robust_kernel_size;    /**< Size of the robust kernel for the keyframes edge */

    // floor coeffs stuff
    double floor_edge_stddev;                   /**< Standard deviation to use for the pose graph information matrix for floor coefficients */
    std::string floor_edge_robust_kernel;       /**< Name of the robust kernel for the floor coefficients edge */
    double floor_edge_robust_kernel_size;       /**< Size of the robust kernel for the floor coefficients edge */
    g2o::VertexPlane* floor_plane_node;         /**< Pose graph node representing the ground */

    // imu stuff
    bool is_imu_accel_enabled;                  /**< Whether or not IMU acceleration is used in the pose graph */
    bool is_imu_orien_enabled;                  /**< Whether or not IMU orientation is used in the pose graph */
    Eigen::Matrix3d imu_to_velo_rot;            /**< Rotation matrix from IMU to Velodyne */
    double imu_accel_edge_stddev;               /**< Standard deviation to use for the pose graph information matrix for acceleration */
    double imu_orien_edge_stddev;               /**< Standard deviation to use for the pose graph information matrix for orientation */
    std::string imu_accel_edge_robust_kernel;   /**< Name of the robust kernel for the acceleration edge */
    double imu_accel_edge_robust_kernel_size;   /**< Size of the robust kernel for the acceleration edge */
    std::string imu_orien_edge_robust_kernel;   /**< Name of the robust kernel for the orientation edge */
    double imu_orien_edge_robust_kernel_size;   /**< Size of the robust kernel for the orientation edge */

    // gps stuff
    bool is_gps_enabled;                        /**< Whether or nor GPS data is used in the pose graph */
    Eigen::Vector3d gps_to_velo_trans;          /**< Translation from GPS to Velodyne */
    boost::optional<Eigen::Vector3d> first_utm; /**< First UTM coordinates received */
    double gps_xy_edge_stddev;                  /**< Standard deviation to use for the pose graph information matrix for planar coordinates */
    double gps_z_edge_stddev;                   /**< Standard deviation to use for the pose graph information matrix for 3D coordinates */
    std::string gps_edge_robust_kernel;         /**< Name of the robust kernel for the GPS edge */
    double gps_edge_robust_kernel_size;         /**< Size of the robust kernel for the GPS edge */

    // loop closure stuff
    std::string loop_edge_robust_kernel;        /**< Name of the robust kernel for the loop edge */
    double loop_edge_robust_kernel_size;        /**< Size of the robust kernel for the loop edge */

    // dispatch queues
    DispatchQueue* insertion_dqueue;
    DispatchQueue* optimization_dqueue;

    // maps adjusters (with base link)
    Eigen::Matrix4f trans_odom2map;

    std::vector<Keyframe::Ptr> keyframes;
    std::unordered_map<uint64_t, Keyframe::Ptr> keyframe_hash;

    GraphHandler* graph_handler;
    InfoMatrixCalculator* info_matrix_calculator;
    LoopDetector* loop_detector;
};


#endif //ARTSLAM_BACKEND_H
