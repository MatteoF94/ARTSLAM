#ifndef ARTSLAM_LASER_3D_BACKEND_HANDLER_H
#define ARTSLAM_LASER_3D_BACKEND_HANDLER_H

#include "observers/keyframe_observers.h"
#include "observers/floor_coefficients_observers.h"
#include "observers/imu_observers.h"
#include "observers/gnss_observers.h"
#include "observers/output_observer.h"
#include <artslam_utils/dispatcher.h>
#include "graph_handler.h"
#include "information_matrix_calculator.h"
#include "loop_detector.h"
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

using namespace artslam::core::utils;

namespace artslam::laser3d {
    class BackendHandler : public KeyframeObserver, public FloorCoefficientsObserver, public IMUObserver, public GNSSObserver {
    public:
        struct Configuration {
            bool use_anchor_graph_node_ = true;                 // whether to use an anchor to tangle the first pose graph node
            double anchor_graph_node_stddev_[6] = {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};
            bool fix_first_graph_node_ = false;                 // whether to fix the first pose graph node
            bool first_graph_node_adaptive_ = true;             // whether to allow the anchor to slightly move around the origin
            std::string odometry_edge_robust_kernel_ = "NONE";  // robust kernel for an odometry edge
            double odometry_edge_robust_kernel_size_ = 1.0;     // robust kernel size for an odometry edge
            double floor_edge_stddev_ = 10.0;                   // stddev for a floor-pose edge
            std::string floor_edge_robust_kernel_ = "NONE";     // robust kernel for a floor-pose edge
            double floor_edge_robust_kernel_size_ = 1.0;        // robust kernel size for a floor-pose edge
            bool imu_acceleration_enabled_ = false;             // whether to use IMU acceleration data
            bool imu_orientation_enabled_ = false;              // whether to use IMU orientation data
            double imu_to_lidar_rotation_[9] = {9.999976e-01, 7.553071e-04, -2.035826e-03, -7.854027e-04, 9.998898e-01, -1.482298e-02, 2.024406e-03, 1.482454e-02, 9.998881e-01};
            double imu_acceleration_stddev_ = 1.0;              // stddev for an IMU acceleration-pose edge
            double imu_orientation_stddev_ = 1.0;               // stddev for an IMU orientation-pose edge
            std::string imu_acceleration_edge_robust_kernel_ = "NONE";  // robust kernel for an IMU acceleration-pose edge
            double imu_acceleration_edge_robust_kernel_size_ = 1.0;     // robust kernel size for an IMU acceleration-pose edge
            std::string imu_orientation_edge_robust_kernel_ = "NONE";   // robust kernel for an IMU orientation-pose edge
            double imu_orientation_edge_robust_kernel_size_ = 1.0;      // robust kernel size for an IMU orientation-pose edge
            bool gnss_enabled_ = true;                          // whether to use GNSS data
            double gnss_to_lidar_translation_[3] = {-0.8086759, 0.3195559, -0.7997231};
            double gnss_edge_xy_stddev_ = 10.0;                 // stddev for a GNSS-pose edge, x and y components
            double gnss_edge_z_stddev_ = 5.0;                   // stddev for a GNSS-pose edge, z component
            std::string gnss_edge_robust_kernel_ = "NONE";      // robust kernel for a GNSS-pose edge
            double gnss_edge_robust_kernel_size_ = 1.0;         // robust kernel size for a GNSS-pose edge
            std::string loop_edge_robust_kernel_ = "Huber";     // robust kernel for a loop closure edge
            double loop_edge_robust_kernel_size_ = 1.0;         // robust kernel size for a loop closure edge
            unsigned short optimizer_iterations_ = 512;         // number of optimizer iterations
            unsigned short maximum_unoptimized_keyframes_ = 2;  // number of keyframes after which to perform optimization
            unsigned short max_keyframes_per_update_ = 10;      // maximum number of processed keyframes per optimization
            bool send_to_observers_ = true;                    // whether to build a map and send it to observers
            bool verbose_ = false;                              // whether the class should be verbose
        };

        BackendHandler();

        explicit BackendHandler(const Configuration& configuration);

        // Signals that a new keyframe has been received
        void update_keyframe_observer(const KeyframeLaser3D::Ptr& keyframe) override;

        void set_gps_to_lidar_translation(EigVector3d g2l_trans);

        // Signals that a new set of floor coefficients has been received
        void update_floor_coefficients_observer(const FloorCoefficients_MSG::ConstPtr& floor_coefficients_msg) override;

        // Signals that a new IMU datum has been received
        void update_raw_imu_observer(const IMU3D_MSG::ConstPtr& imu3d_msg) override;

        // Signals that a new GNSS datum has been received
        void update_raw_gnss_observer(const GeoPointStamped_MSG::ConstPtr& gnss_msg) override;

        void set_graph_handler(GraphHandler* graph_handler);

        void set_information_matrix_calculator(InformationMatrixCalculator* information_matrix_calculator);

        void set_loop_detector(LoopDetector* loop_detector);

        // Registers an object waiting for filtered point clouds
        void register_slam_output_observer(SlamOutputObserver* filtered_pointcloud_observer);

        // Removes an object waiting for filtered point clouds
        void remove_slam_output_observer(SlamOutputObserver* filtered_pointcloud_observer);

        void reinforce_and_optimize();

        bool save_results(const std::string& results_path);

        // testing purposes
        uint64_t total_time_ = 0;
        uint32_t count_ = 0;

    private:
        // Notifies all the objects waiting for filtered point clouds
        void notify_slam_output_observers(const pcl::PointCloud<Point3I>::Ptr& map, std::vector<EigIsometry3d>);

        void insert_keyframe(const KeyframeLaser3D::Ptr& keyframe);

        void insert_floor_coefficients_msg(const FloorCoefficients_MSG::ConstPtr& floor_coefficients_msg);

        void insert_imu_msg(const IMU3D_MSG::ConstPtr& imu3d_msg);

        void insert_gnss_msg(const GeoPointStamped_MSG::ConstPtr& gnss_msg);

        bool flush_incoming_keyframes();

        bool flush_incoming_floor_coefficients();

        bool flush_incoming_imu3d_msgs();

        bool flush_incoming_gnss_msgs();

        bool detect_loops();

        void prepare_data_for_visualization();

        Configuration configuration_;

        std::deque<KeyframeLaser3D::Ptr> new_keyframes_;
        std::deque<KeyframeLaser3D::Ptr> incoming_keyframes_;
        std::vector<KeyframeLaser3D::Ptr> keyframes_;
        std::map<uint64_t, KeyframeLaser3D::Ptr> keyframe_hash_;

        // Keyframe variables
        g2o::VertexSE3* anchor_graph_node_ = nullptr;
        g2o::EdgeSE3* anchor_graph_edge_ = nullptr;

        // Floor coefficient variables
        g2o::VertexPlane* floor_plane_node_ = nullptr;
        std::mutex floor_coefficients_mutex_;
        std::deque<FloorCoefficients_MSG::ConstPtr> floor_coefficients_msgs_;
        bool inclined_floor_ = false;

        // 3D IMU variables
        std::mutex imu3d_mutex_;
        std::deque<IMU3D_MSG::ConstPtr> imu3d_msgs_;
        std::optional<EigQuaterniond> first_imu_;

        // GNSS variables
        std::mutex gnss_mutex_;
        std::deque<GeoPointStamped_MSG::ConstPtr> gnss_msgs_;
        std::optional<EigVector3d> first_utm_;

        std::mutex keyframe_mutex_;
        std::mutex transformation_odom2map_mutex_;
        std::mutex optimization_mutex_;

        EigMatrix4d transformation_odom2map_ = EigMatrix4d::Identity();

        std::unique_ptr<Dispatcher> insertion_dispatcher_;
        std::unique_ptr<Dispatcher> optimization_dispatcher_;

        GraphHandler* graph_handler_ = nullptr;
        InformationMatrixCalculator* information_matrix_calculator_ = nullptr;
        LoopDetector* loop_detector_ = nullptr;

        std::vector<SlamOutputObserver*> slam_output_observers_;
    };
}


#endif //ARTSLAM_LASER_3D_BACKEND_HANDLER_H
