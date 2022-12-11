#include "backend_handler.h"
#include <artslam_g2o/edge_se3_plane.hpp>
#include <artslam_g2o/edge_se3_priorvec.hpp>
#include <artslam_g2o/edge_se3_priorquat.hpp>
#include <artslam_g2o/edge_se3_priorxy.hpp>
#include <artslam_g2o/edge_se3_priorxyz.hpp>
#include <artslam_utils/types_converter.h>

#include <pcl/io/pcd_io.h>        
#include <pcl/filters/voxel_grid.h>

using namespace artslam::laser3d;

BackendHandler::BackendHandler() {
    std::stringstream msg;

    if(configuration_.verbose_) {
        msg << "[BackendHandler] Creating and configuring the back-end handler\n";
        std::cout << msg.str();
        msg.str("");
    }

    insertion_dispatcher_ = std::make_unique<Dispatcher>("BackendHandlerInsertionDispatcher", 1);
    optimization_dispatcher_ = std::make_unique<Dispatcher>("BackendHandlerOptimizationDispatcher", 1);

    occupancygrid_ = std::make_shared<OccupancyGrid>();
    occmap_width_ = configuration_.occmap_width_;
    occmap_height_ = configuration_.occmap_height_;
    occmap_bbox_[0] = - configuration_.occmap_resolution_ * static_cast<float>(occmap_width_ / 2.0);
    occmap_bbox_[1] = configuration_.occmap_resolution_ * static_cast<float>(occmap_width_ / 2.0);
    occmap_bbox_[2] = - configuration_.occmap_resolution_ * static_cast<float>(occmap_height_ / 2.0);
    occmap_bbox_[3] = configuration_.occmap_resolution_ * static_cast<float>(occmap_height_ / 2.0);
    occupancygrid_->width_ = occmap_width_;
    occupancygrid_->height_ = occmap_height_;
    occupancygrid_->resolution_ = configuration_.occmap_resolution_;
    occupancygrid_->data_.resize(occmap_width_ * occmap_height_, -1);
    occupancygrid_->initial_pose_ = EigIsometry3d::Identity();
    occupancygrid_->initial_pose_.translation() = EigVector3d(occmap_bbox_[0], occmap_bbox_[2], 0);

    if(configuration_.verbose_) {
        msg << "[BackendHandler] Finished creating and configuring the back-end handler\n";
        std::cout << msg.str();
    }
}

BackendHandler::BackendHandler(const Configuration &configuration) {
    std::stringstream msg;
    configuration_.verbose_ = configuration.verbose_;

    if(configuration_.verbose_) {
        msg << "[BackendHandler] Creating and configuring the back-end handler\n";
        std::cout << msg.str();
        msg.str("");
    }

    // keyframes parameters
    configuration_.use_anchor_graph_node_ = configuration.use_anchor_graph_node_;
    for(int i = 0; i < 6; i++)
        configuration_.anchor_graph_node_stddev_[i] = configuration.anchor_graph_node_stddev_[i];
    configuration_.fix_first_graph_node_ = configuration.fix_first_graph_node_;
    configuration_.first_graph_node_adaptive_ = configuration.first_graph_node_adaptive_;
    configuration_.odometry_edge_robust_kernel_ = configuration.odometry_edge_robust_kernel_;
    configuration_.odometry_edge_robust_kernel_size_ = configuration.odometry_edge_robust_kernel_size_;

    // floor coefficient variables
    configuration_.floor_edge_stddev_ = configuration.floor_edge_stddev_;
    configuration_.floor_edge_robust_kernel_ = configuration.floor_edge_robust_kernel_;
    configuration_.floor_edge_robust_kernel_size_ = configuration.floor_edge_robust_kernel_size_;

    // 3D IMU variables
    configuration_.imu_acceleration_enabled_ = configuration.imu_acceleration_enabled_;
    configuration_.imu_orientation_enabled_ = configuration.imu_orientation_enabled_;
    for(int i = 0; i < 9; i++)
        configuration_.imu_to_lidar_rotation_[i] = configuration.imu_to_lidar_rotation_[i];
    configuration_.imu_acceleration_stddev_ = configuration.imu_acceleration_stddev_;
    configuration_.imu_orientation_stddev_ = configuration.imu_orientation_stddev_;
    configuration_.imu_acceleration_edge_robust_kernel_ = configuration.imu_acceleration_edge_robust_kernel_;
    configuration_.imu_acceleration_edge_robust_kernel_size_ = configuration.imu_acceleration_edge_robust_kernel_size_;
    configuration_.imu_orientation_edge_robust_kernel_ = configuration.imu_orientation_edge_robust_kernel_;
    configuration_.imu_orientation_edge_robust_kernel_size_ = configuration.imu_orientation_edge_robust_kernel_size_;

    // GNSS variables
    configuration_.gnss_enabled_ = configuration.gnss_enabled_;
    for(int i = 0; i < 3; i++)
        configuration_.gnss_to_lidar_translation_[i] = configuration.gnss_to_lidar_translation_[i];
    configuration_.gnss_edge_xy_stddev_ = configuration.gnss_edge_xy_stddev_;
    configuration_.gnss_edge_z_stddev_ = configuration.gnss_edge_z_stddev_;
    configuration_.gnss_edge_robust_kernel_ = configuration.gnss_edge_robust_kernel_;
    configuration_.gnss_edge_robust_kernel_size_ = configuration.gnss_edge_robust_kernel_size_;

    // loop detection variables
    configuration_.loop_edge_robust_kernel_ = configuration.loop_edge_robust_kernel_;
    configuration_.loop_edge_robust_kernel_size_ = configuration.loop_edge_robust_kernel_size_;

    // optimization parameters
    configuration_.optimizer_iterations_ = configuration.optimizer_iterations_;
    configuration_.maximum_unoptimized_keyframes_ = configuration.maximum_unoptimized_keyframes_;
    configuration_.max_keyframes_per_update_ = configuration.max_keyframes_per_update_;

    configuration_.send_to_observers_ = configuration.send_to_observers_;

    insertion_dispatcher_ = std::make_unique<Dispatcher>("BackendHandlerInsertionDispatcher", 1);
    optimization_dispatcher_ = std::make_unique<Dispatcher>("BackendHandlerOptimizationDispatcher", 1);

    occupancygrid_ = std::make_shared<OccupancyGrid>();
    occmap_width_ = configuration_.occmap_width_;
    occmap_height_ = configuration_.occmap_height_;
    occmap_bbox_[0] = - configuration_.occmap_resolution_ * static_cast<float>(occmap_width_ / 2.0);
    occmap_bbox_[1] = configuration_.occmap_resolution_ * static_cast<float>(occmap_width_ / 2.0);
    occmap_bbox_[2] = - configuration_.occmap_resolution_ * static_cast<float>(occmap_height_ / 2.0);
    occmap_bbox_[3] = configuration_.occmap_resolution_ * static_cast<float>(occmap_height_ / 2.0);
    occupancygrid_->width_ = occmap_width_;
    occupancygrid_->height_ = occmap_height_;
    occupancygrid_->resolution_ = configuration_.occmap_resolution_;
    occupancygrid_->data_.resize(occmap_width_ * occmap_height_, -1);
    occupancygrid_->initial_pose_ = EigIsometry3d::Identity();
    occupancygrid_->initial_pose_.translation() = EigVector3d(occmap_bbox_[0], occmap_bbox_[2], 0);

    if(configuration_.verbose_) {
        msg << "[BackendHandler] Finished creating and configuring the back-end handler\n";
        std::cout << msg.str();
    }
}

// Registers an object waiting for filtered point clouds
void BackendHandler::register_slam_output_observer(SlamOutputObserver *filtered_pointcloud_observer) {
    slam_output_observers_.emplace_back(filtered_pointcloud_observer);
}

// Removes an object waiting for filtered point clouds
void BackendHandler::remove_slam_output_observer(SlamOutputObserver *filtered_pointcloud_observer) {
    auto iterator = std::find(slam_output_observers_.begin(), slam_output_observers_.end(), filtered_pointcloud_observer);

    if(iterator != slam_output_observers_.end()) {
        slam_output_observers_.erase(iterator);
    }
}

// Notifies all the objects waiting for filtered point clouds
void BackendHandler::notify_slam_output_observers(const pcl::PointCloud<Point3I>::Ptr& map, std::vector<EigIsometry3d> poses) {
    for(SlamOutputObserver* observer : slam_output_observers_) {
        observer->update_slam_output_observer(map, poses, occupancygrid_);
    }
}

void BackendHandler::update_keyframe_observer(const KeyframeLaser3D::Ptr& keyframe) {
    insertion_dispatcher_->dispatch([this, keyframe]{insert_keyframe(keyframe);});
}

void BackendHandler::update_floor_coefficients_observer(const FloorCoefficients_MSG::ConstPtr &floor_coefficients_msg) {
    insertion_dispatcher_->dispatch([this, floor_coefficients_msg]{insert_floor_coefficients_msg(floor_coefficients_msg);});
}

void BackendHandler::update_raw_imu_observer(const IMU3D_MSG::ConstPtr& imu_msg) {
    insertion_dispatcher_->dispatch([this, imu_msg]{ insert_imu_msg(imu_msg);});
}

void BackendHandler::update_raw_gnss_observer(const GeoPointStamped_MSG::ConstPtr &gnss_msg) {
    insertion_dispatcher_->dispatch([this, gnss_msg]{insert_gnss_msg(gnss_msg);});
}

void BackendHandler::set_graph_handler(GraphHandler *graph_handler) {
    graph_handler_ = graph_handler;
}

void BackendHandler::set_information_matrix_calculator(InformationMatrixCalculator *information_matrix_calculator) {
    information_matrix_calculator_ = information_matrix_calculator;
}

void BackendHandler::set_loop_detector(LoopDetector *loop_detector) {
    loop_detector_ = loop_detector;
}

void BackendHandler::insert_keyframe(const KeyframeLaser3D::Ptr &keyframe) {
    std::stringstream msg;
    std::lock_guard<std::mutex> insertion_lock(keyframe_mutex_);
    if(configuration_.verbose_) {
        msg << "[BackendHandler] Incoming keyframe with cloud sequence: " << keyframe->pointcloud_->header.seq << "\n";
        std::cout << msg.str();
        msg.str("");
    }
    incoming_keyframes_.emplace_back(keyframe);

    if(incoming_keyframes_.size() >= configuration_.maximum_unoptimized_keyframes_) {
        optimization_dispatcher_->dispatch([this]{reinforce_and_optimize();});
    }
}

void BackendHandler::insert_floor_coefficients_msg(const FloorCoefficients_MSG::ConstPtr &floor_coefficients_msg) {
    std::stringstream msg;
    std::lock_guard<std::mutex> insertion_lock(floor_coefficients_mutex_);
    if(configuration_.verbose_) {
        msg << "[BackendHandler] Incoming floor coefficients message, with ID: " << floor_coefficients_msg->header_.sequence_ << "\n";
        std::cout << msg.str();
        msg.str("");
    }

    floor_coefficients_msgs_.emplace_back(floor_coefficients_msg);
}

void BackendHandler::insert_imu_msg(const IMU3D_MSG::ConstPtr &imu3d_msg) {
    std::stringstream msg;
    std::lock_guard<std::mutex> insertion_lock(imu3d_mutex_);
    if(configuration_.verbose_) {
        msg << "[BackendHandler] Incoming IMU 3D message, with ID: " << imu3d_msg->header_.sequence_ << "\n";
        std::cout << msg.str();
        msg.str("");
    }

    // if imu is not enabled, return immediately
    if(!configuration_.imu_acceleration_enabled_ && configuration_.imu_orientation_enabled_) return;

    imu3d_msgs_.emplace_back(imu3d_msg);
}

void BackendHandler::insert_gnss_msg(const GeoPointStamped_MSG::ConstPtr &gnss_msg) {
    std::stringstream msg;
    std::lock_guard<std::mutex> insertion_lock(gnss_mutex_);
    if(configuration_.verbose_) {
        msg << "[BackendHandler] Incoming GNSS 3D message, with ID: " << gnss_msg->header_.sequence_ << "\n";
        std::cout << msg.str();
        msg.str("");
    }

    // if gnss is not enabled, return immediately
    if(!configuration_.gnss_enabled_) return;

    gnss_msgs_.emplace_back(gnss_msg);
}

void BackendHandler::reinforce_and_optimize() {
    std::stringstream msg;
    auto start = std::chrono::high_resolution_clock::now();

    std::lock_guard<std::mutex> optimization_lock(optimization_mutex_);

    bool has_new_keyframes = flush_incoming_keyframes();
    bool has_loops = detect_loops();
    bool has_new_floor_coefficients = flush_incoming_floor_coefficients();
    bool has_new_imu3d_msgs = flush_incoming_imu3d_msgs();
    bool has_new_gnss_msgs = flush_incoming_gnss_msgs();

    if(!has_new_keyframes & !has_new_floor_coefficients & !has_new_imu3d_msgs & !has_new_gnss_msgs) {
        return;
    }

    // move the anchor node to the first node, allowing the latter to move freely around the origin of the map
    if(anchor_graph_node_ && configuration_.first_graph_node_adaptive_) {
        EigIsometry3d anchor_target = dynamic_cast<g2o::VertexSE3*>(anchor_graph_edge_->vertices()[1])->estimate();
        anchor_graph_node_->setEstimate(anchor_target);
    }

    // optimize the graph only under certain conditions
    if(has_new_floor_coefficients || has_new_imu3d_msgs || has_new_gnss_msgs || has_loops) {
        graph_handler_->optimize(configuration_.optimizer_iterations_);
    }

    // update the transformation between odometry and map
    const auto& last_keyframe = keyframes_.back();
    EigIsometry3d transformation = last_keyframe->graph_node_->estimate() * last_keyframe->odometry_.inverse();
    transformation_odom2map_mutex_.lock();
    transformation_odom2map_ = transformation.matrix();
    transformation_odom2map_mutex_.unlock();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    total_time_ += duration.count();
    count_++;

    //std::cout << "VERT EDGE - " << graph_handler_->num_vertices() << " " << graph_handler_->num_edges() << "\n";

    if(configuration_.send_to_observers_) {
        prepare_data_for_visualization();
    }

    if(configuration_.verbose_) {
        msg << "[LoopDetector] Total time and count: " << total_time_ << ", " << count_ << "\n";
        std::cout << msg.str();
    }
    //graph_handler_->save("/home/matteo/pure_lidar_graph.g2o");
}

bool BackendHandler::flush_incoming_keyframes() {
    std::lock_guard<std::mutex> keyframe_lock(keyframe_mutex_);

    // if there are no incoming keyframes, return immediately (should never happen though)
    if(incoming_keyframes_.empty()) {
        return false;
    }

    // copy the odometry 2 map transformation, to use it locally
    transformation_odom2map_mutex_.lock();
    Eigen::Isometry3d transformation_odom2map(transformation_odom2map_);
    transformation_odom2map_mutex_.unlock();

    // insert the keyframes in the back-end and pose graph
    int num_keyframe_flushed_ = 0;
    for(int i = 0; i < std::min<int>(incoming_keyframes_.size(), configuration_.max_keyframes_per_update_); ++i) {
        KeyframeLaser3D::Ptr keyframe = incoming_keyframes_[i];
        new_keyframes_.emplace_back(keyframe);
        num_keyframe_flushed_ = i;
        loop_detector_->make_scancontext(keyframe->pointcloud_);

        for(int j = 0; j < keyframe->pointcloud_->size(); j++) {
            const Point3I *p = &keyframe->pointcloud_->points[j];
            if(p->z >= -0.05 && p->z <= 0.05) {
                Point3I np(*p);
                np.z = 0;
                keyframe->pointcloud_2d_->emplace_back(np);
            }
        }

        EigIsometry3d odometry = transformation_odom2map * keyframe->odometry_;
        keyframe->graph_node_ = graph_handler_->add_se3_node(odometry);
        keyframe_hash_[keyframe->timestamp_] = keyframe;

        // when inserting the first keyframe, optionally insert an anchor fixed node
        if(this->keyframes_.empty() && new_keyframes_.size() == 1) {
            if (configuration_.use_anchor_graph_node_) {
                EigMatrixXd information_matrix = EigMatrixXd::Identity(6, 6);
                for (int j = 0; j < 6; j++) {
                    double stddev = 1.0;
                    if(configuration_.anchor_graph_node_stddev_[i] != -1) {
                        stddev = configuration_.anchor_graph_node_stddev_[i];
                    }
                    information_matrix(j,j) = 1.0/stddev;
                }

                anchor_graph_node_ = graph_handler_->add_se3_node(EigIsometry3d::Identity());
                anchor_graph_node_->setFixed(true);
                anchor_graph_edge_ = graph_handler_->add_se3_edge(anchor_graph_node_, keyframe->graph_node_, EigIsometry3d::Identity(), information_matrix);
            }

            if(configuration_.fix_first_graph_node_) {
                keyframe->graph_node_->setFixed(true);
            }
        }

        if(i == 0 && keyframes_.empty()) continue;

        const KeyframeLaser3D::Ptr previous_keyframe_ = i == 0 ? keyframes_.back() : incoming_keyframes_[i - 1];
        EigIsometry3d relative_pose = keyframe->odometry_.inverse() * previous_keyframe_->odometry_;
        // TODO give the choice to re-compute the fitness
        EigMatrixXd information_matrix = information_matrix_calculator_->compute_information_matrix(keyframe->reliability_);
        g2o::EdgeSE3* edge = graph_handler_->add_se3_edge(keyframe->graph_node_, previous_keyframe_->graph_node_, relative_pose, information_matrix);
        graph_handler_->add_robust_kernel(edge, configuration_.odometry_edge_robust_kernel_, configuration_.odometry_edge_robust_kernel_size_);
    }

    incoming_keyframes_.erase(incoming_keyframes_.begin(), incoming_keyframes_.begin() + num_keyframe_flushed_ + 1);
    return true;
}

bool BackendHandler::flush_incoming_floor_coefficients() {
    std::lock_guard<std::mutex> floor_coefficients_lock(floor_coefficients_mutex_);

    // if there are no keyframes, return immediately (should never happen though)
    if(keyframes_.empty() || inclined_floor_) {
        return false;
    }

    const uint64_t latest_keyframe_timestamp = keyframes_.back()->timestamp_;

    bool updated = false;
    for(const auto& floor_coefficients : floor_coefficients_msgs_) {
        if(floor_coefficients->header_.timestamp_ > latest_keyframe_timestamp) {
            break;
        }

        // finding the floor coefficients is easy, as they are extracted from point clouds and
        // share the same timestamp
        auto found = keyframe_hash_.find(floor_coefficients->header_.timestamp_);
        if(found == keyframe_hash_.end()) {
            continue;
        }

        const auto& keyframe = found->second;
        long keyframe_idx = std::distance(keyframe_hash_.begin(), found);
        if(keyframe_hash_.size() > 1) {
            auto keyframes_it = keyframe_hash_.begin();
            std::advance(keyframes_it, keyframe_idx - 1);
            const auto& prev_keyframe = keyframes_it->second;

            EigIsometry3d transform = prev_keyframe->odometry_.inverse() * keyframe->odometry_;
            EigVector3d vertical_normal = transform.rotation().col(2);
            double dot = vertical_normal.normalized().dot(EigVector3d::UnitZ());
            if(std::abs(dot) < std::cos(15 * M_PI / 180.0)) {
                inclined_floor_ = true;
                break;
            }
        }

        if(!floor_plane_node_) {
            floor_plane_node_ = graph_handler_->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
            floor_plane_node_->setFixed(true);
        }

        EigVector4d coefficients(floor_coefficients->coefficients_[0], floor_coefficients->coefficients_[1], floor_coefficients->coefficients_[2], floor_coefficients->coefficients_[3]);
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity() * (1.0 / configuration_.floor_edge_stddev_);
        g2o::EdgeSE3Plane* edge = graph_handler_->add_se3_plane_edge(keyframe->graph_node_, floor_plane_node_, coefficients, information_matrix);
        graph_handler_->add_robust_kernel(edge, configuration_.floor_edge_robust_kernel_, configuration_.floor_edge_robust_kernel_size_);

        keyframe->floor_coefficients_ = coefficients;

        updated = true;
    }

    auto remove_loc = std::upper_bound(floor_coefficients_msgs_.begin(), floor_coefficients_msgs_.end(), latest_keyframe_timestamp, [=](uint64_t timestamp, const FloorCoefficients_MSG::ConstPtr& coeffs) { return coeffs->header_.timestamp_ > timestamp; });
    floor_coefficients_msgs_.erase(floor_coefficients_msgs_.begin(), remove_loc);

    return updated;
}

bool BackendHandler::flush_incoming_imu3d_msgs() {
    std::lock_guard<std::mutex> imu3d_lock(imu3d_mutex_);

    if(!configuration_.imu_acceleration_enabled_ && !configuration_.imu_orientation_enabled_)
        return false;

    if(keyframes_.empty() || imu3d_msgs_.empty())
        return false;

    bool updated = false;
    auto imu3d_msg_it = imu3d_msgs_.begin();

    for(auto& keyframe : keyframes_) {
        uint64_t keyframe_timestamp = keyframe->timestamp_;

        // if the keyframe is older than the last IMU 3D message, stop the data association
        if(keyframe_timestamp > imu3d_msgs_.back()->header_.timestamp_) {
            break;
        }

        // if the keyframe is newer than the current IMU 3D message considered, skip this cycle
        if(keyframe_timestamp < (*imu3d_msg_it)->header_.timestamp_) {
            continue;
        }

        // find the closest IMU 3D message, in time, to the keyframes
        auto closest_imu = imu3d_msg_it;
        for(auto imu = imu3d_msg_it; imu != imu3d_msgs_.end(); imu++) {
            uint64_t dt = ((*closest_imu)->header_.timestamp_ > keyframe_timestamp) ? ((*closest_imu)->header_.timestamp_ - keyframe_timestamp) : (keyframe_timestamp - (*closest_imu)->header_.timestamp_);
            uint64_t dt2 = ((*imu)->header_.timestamp_ > keyframe_timestamp) ? ((*imu)->header_.timestamp_ - keyframe_timestamp) : (keyframe_timestamp - (*imu)->header_.timestamp_);
            if(dt < dt2) {
                break;
            }

            closest_imu = imu;
        }

        imu3d_msg_it = closest_imu;
        uint64_t dt = ((*closest_imu)->header_.timestamp_ > keyframe_timestamp) ? ((*closest_imu)->header_.timestamp_ - keyframe_timestamp) : (keyframe_timestamp - (*closest_imu)->header_.timestamp_);
        uint64_t limit = 100000000;
        if(dt > limit) {
            continue;
        }

        EigMatrix3d imu_to_lidar_rotation;
        imu_to_lidar_rotation << configuration_.imu_to_lidar_rotation_[0], configuration_.imu_to_lidar_rotation_[1], configuration_.imu_to_lidar_rotation_[2],
                configuration_.imu_to_lidar_rotation_[3], configuration_.imu_to_lidar_rotation_[4], configuration_.imu_to_lidar_rotation_[5],
                configuration_.imu_to_lidar_rotation_[6], configuration_.imu_to_lidar_rotation_[7], configuration_.imu_to_lidar_rotation_[8];

        // TODO should check it
        /*if(imu_acceleration_enabled_ && (*closest_imu)->has_linear_acceleration_) {
            const EigVector3d acceleration = imu_to_lidar_rotation * (*closest_imu)->linear_acceleration_;
            keyframe->acceleration_ = acceleration;
            EigMatrixXd information_matrix = EigMatrixXd::Identity(3, 3) / imu_acceleration_stddev_;
            g2o::EdgeSE3PriorVec* edge = graph_handler_->add_se3_prior_vec_edge(keyframe->graph_node_, -EigVector3d::UnitZ(), *keyframe->acceleration_, information_matrix);
            graph_handler_->add_robust_kernel(edge, imu_acceleration_edge_robust_kernel_, imu_acceleration_edge_robust_kernel_size_);
        }*/

        if(configuration_.imu_orientation_enabled_ && (*closest_imu)->has_orientation_) {
            EigQuaterniond imu_to_lidar_rotation_quat;
            imu_to_lidar_rotation_quat = EigMatrix3d(imu_to_lidar_rotation);    // Eigen automatically converts rotation forms
            const EigQuaterniond orientation = imu_to_lidar_rotation_quat * (*closest_imu)->orientation_;
            keyframe->orientation_ = orientation;
            if(keyframe->orientation_->w() < 0.0) {
                keyframe->orientation_->coeffs() = -keyframe->orientation_->coeffs();
            }
            EigQuaterniond odom_quat(keyframe->odometry_.linear());
            Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / configuration_.imu_orientation_stddev_;
            g2o::EdgeSE3PriorQuat* edge = graph_handler_->add_se3_prior_quat_edge(keyframe->graph_node_, *keyframe->orientation_, info);
            graph_handler_->add_robust_kernel(edge, configuration_.imu_orientation_edge_robust_kernel_, configuration_.imu_orientation_edge_robust_kernel_size_);
        }
        updated = true;
    }

    auto remove_loc = std::upper_bound(imu3d_msgs_.begin(), imu3d_msgs_.end(), keyframes_.back()->timestamp_, [=](uint64_t timestamp, const IMU3D_MSG::ConstPtr& imu3d_msg) { return imu3d_msg->header_.timestamp_ > timestamp; });
    imu3d_msgs_.erase(imu3d_msgs_.begin(), remove_loc);

    return updated;
}

void BackendHandler::set_gps_to_lidar_translation(EigVector3d g2l_trans) {
    configuration_.gnss_to_lidar_translation_[0] = g2l_trans.x();
    configuration_.gnss_to_lidar_translation_[1] = g2l_trans.y();
    configuration_.gnss_to_lidar_translation_[2] = g2l_trans.z();
}

bool BackendHandler::flush_incoming_gnss_msgs() {
    std::lock_guard<std::mutex> gnss_lock(gnss_mutex_);
    if(keyframes_.empty() || gnss_msgs_.empty() || !configuration_.gnss_enabled_)
        return false;
    bool updated = false;
    auto gnss_msg_it = gnss_msgs_.begin();
    for(auto& keyframe : keyframes_) {
        uint64_t keyframe_timestamp = keyframe->timestamp_;
        // if the keyframe is older than the last GNSS message, stop the data association
        if(keyframe_timestamp > gnss_msgs_.back()->header_.timestamp_ + 10000000) {
            break;
        }
        // if the keyframe is newer than the current GNSS message considered, skip this cycle
        if(keyframe_timestamp < (*gnss_msg_it)->header_.timestamp_ - 10000000) {
            continue;
        }
        // find the closest GNSS message, in time, to the keyframes
        auto closest_gnss = gnss_msg_it;
        for(auto gnss = gnss_msg_it; gnss != gnss_msgs_.end(); gnss++) {
            uint64_t dt = ((*closest_gnss)->header_.timestamp_ > keyframe_timestamp) ? ((*closest_gnss)->header_.timestamp_ - keyframe_timestamp) : (keyframe_timestamp - (*closest_gnss)->header_.timestamp_);
            uint64_t dt2 = ((*gnss)->header_.timestamp_ > keyframe_timestamp) ? ((*gnss)->header_.timestamp_ - keyframe_timestamp) : (keyframe_timestamp - (*gnss)->header_.timestamp_);
            if(dt < dt2) {
                break;
            }
            closest_gnss = gnss;
        }
        gnss_msg_it = closest_gnss;
        uint64_t dt = ((*closest_gnss)->header_.timestamp_ > keyframe_timestamp) ? ((*closest_gnss)->header_.timestamp_ - keyframe_timestamp) : (keyframe_timestamp - (*closest_gnss)->header_.timestamp_);
        uint64_t limit = 200000000;
        if(dt > limit) {
            continue;
        }
        // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
        double northing;
        double easting;
        int zone;
        char band;
        TypesConverter::LL_to_UTM((*closest_gnss)->latitude_, (*closest_gnss)->longitude_, northing, easting, zone, band);
        EigVector3d xyz(easting, northing, (*closest_gnss)->altitude_);
        std::string gpspath= "/home/alessandro/easymile/src/easymile/result/gpsart.csv";
        std::ofstream gpsfile(gpspath.c_str(), std::ofstream::app);
        gpsfile << /*"GPS: " << std::setprecision(9) << xyz.transpose() << */ std::setprecision(15) << (*closest_gnss)->latitude_ << " " << (*closest_gnss)->longitude_ << " " << (*closest_gnss)->altitude_ << " " << (*closest_gnss)->header_.timestamp_ << "\n";
        keyframe->gps_utm_coordinates_ = xyz;
        if(!keyframe->utm_compensated_ && !keyframe->orientation_)
            continue;
        EigVector3d g2l_translation = EigVector3d(configuration_.gnss_to_lidar_translation_[0], configuration_.gnss_to_lidar_translation_[1], configuration_.gnss_to_lidar_translation_[2]);
        // if the keyframe has IMU information, use it to estimate the LiDAR position in UTM
        EigVector3d lidar_utm;
        if(keyframe->orientation_) {
            lidar_utm = xyz - keyframe->orientation_.value() * g2l_translation;
            keyframe->lidar_utm_coordinates_ = lidar_utm;
        } else {
            // if the keyframe is compensated w.r.t. East and North, use its rotation matrix to estimate the LiDAR position in UTM
            EigMatrix3d rotation = keyframe->odometry_.linear();
            EigQuaterniond quat(rotation);
            lidar_utm = xyz - quat * g2l_translation;
            keyframe->lidar_utm_coordinates_ = lidar_utm;
        }
        // the first GNSS position will be the origin of the map
        if(!first_utm_) {
            first_utm_ = keyframe->lidar_utm_coordinates_;
        }
        lidar_utm -= (*first_utm_);   // convert in ENU coordinates
        // TODO if floor is not used, can use altitude instead
        lidar_utm.z() = 0;    // not really interested in altitude because of the floor coefficients
        keyframe->lidar_enu_coordinates_ = lidar_utm;

        g2o::OptimizableGraph::Edge* edge;
        if(lidar_utm.z() == 0) {
            EigMatrix2d information_matrix = EigMatrix2d::Identity();
            if((*closest_gnss)->covariance_type_ == 0)
                information_matrix /= configuration_.gnss_edge_xy_stddev_;
            else {
                EigMatrix3d covariance_matrix;
                covariance_matrix << (*closest_gnss)->covariance_[0], (*closest_gnss)->covariance_[1], (*closest_gnss)->covariance_[2],
                                     (*closest_gnss)->covariance_[3], (*closest_gnss)->covariance_[4], (*closest_gnss)->covariance_[5],
                                     (*closest_gnss)->covariance_[6], (*closest_gnss)->covariance_[7], (*closest_gnss)->covariance_[8];
                // TODO not sure if a simple matrix reduction is enough 
                information_matrix = covariance_matrix.inverse().block<2,2>(0,0);
            }

            edge = graph_handler_->add_se3_prior_xy_edge(keyframe->graph_node_, lidar_utm.head<2>(), information_matrix);
        } else {
            Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
            if((*closest_gnss)->covariance_type_ == 0) {
                information_matrix.block<2, 2>(0, 0) /= configuration_.gnss_edge_xy_stddev_;
                information_matrix(2, 2) /= configuration_.gnss_edge_z_stddev_;
            }
            else {
                EigMatrix3d covariance_matrix;
                covariance_matrix << (*closest_gnss)->covariance_[0], (*closest_gnss)->covariance_[1], (*closest_gnss)->covariance_[2],
                                     (*closest_gnss)->covariance_[3], (*closest_gnss)->covariance_[4], (*closest_gnss)->covariance_[5],
                                     (*closest_gnss)->covariance_[6], (*closest_gnss)->covariance_[7], (*closest_gnss)->covariance_[8];
                information_matrix = covariance_matrix.inverse();
            }
            edge = graph_handler_->add_se3_prior_xyz_edge(keyframe->graph_node_, lidar_utm, information_matrix);
        }
        graph_handler_->add_robust_kernel(edge, configuration_.gnss_edge_robust_kernel_, configuration_.gnss_edge_robust_kernel_size_);
        updated = true;
    }
    auto remove_loc = std::upper_bound(gnss_msgs_.begin(), gnss_msgs_.end(), keyframes_.back()->timestamp_, [=](uint64_t timestamp, const GeoPointStamped_MSG::ConstPtr& gnss_msg) { return gnss_msg->header_.timestamp_ > timestamp; });
    gnss_msgs_.erase(gnss_msgs_.begin(), remove_loc);
    return updated;
}


bool BackendHandler::detect_loops() {
    std::vector<Loop::Ptr> loops = loop_detector_->detect(keyframes_, new_keyframes_);

    for (const Loop::Ptr &loop: loops) {
            EigIsometry3d relative_pose(loop->relative_pose_.cast<double>());
            EigMatrixXd information_matrix = information_matrix_calculator_->compute_information_matrix(
                    loop->reliability_);

            g2o::EdgeSE3 *edge = graph_handler_->add_se3_edge(loop->keyframe_source_->graph_node_,
                                                              loop->keyframe_target_->graph_node_, relative_pose,
                                                              information_matrix);
            graph_handler_->add_robust_kernel(edge, configuration_.loop_edge_robust_kernel_, configuration_.loop_edge_robust_kernel_size_);
    }

    std::copy(new_keyframes_.begin(), new_keyframes_.end(), std::back_inserter(keyframes_));
    new_keyframes_.clear();

    if(loops.empty())
        return false;
    else
        return true;
}

bool BackendHandler::save_results(const std::string& results_path) {
    std::lock_guard<std::mutex> lock(optimization_mutex_);
    std::string indfile = results_path + "indices.txt";
    std::string odomfile = results_path + "odometries.txt";
    std::string mapfile = results_path + "map.pcd";
    std::string graphfile = results_path + "graph.g2o";

    std::ofstream indoutfile(indfile.c_str());
    std::ofstream odomoutfile(odomfile.c_str());

    pcl::PointCloud<Point3I>::Ptr map(new pcl::PointCloud<Point3I>());

    for(const KeyframeLaser3D::Ptr& keyframe : keyframes_) {
        indoutfile << keyframe->pointcloud_->header.seq << "\n";

        EigIsometry3d pose = keyframe->graph_node_->estimate();
        EigMatrix4d posem = pose.matrix();
        odomoutfile << posem(0,0) << " " << posem(0,1) << " " << posem(0,2) << " " << posem(0,3) << " " <<
                       posem(1,0) << " " << posem(1,1) << " " << posem(1,2) << " " << posem(1,3) << " " <<
                       posem(2,0) << " " << posem(2,1) << " " << posem(2,2) << " " << posem(2,3) << " " << keyframe->pointcloud_->header.stamp <<"\n";

        pcl::PointCloud<Point3I>::Ptr transformed_pointcloud(new pcl::PointCloud<Point3I>());
        pcl::transformPointCloud(*(keyframe->pointcloud_), *transformed_pointcloud, posem);
        *map += *transformed_pointcloud;
    }

    map->width = map->size();
    map->height = 1;
    map->is_dense = false;

    pcl::VoxelGrid<Point3I> downsample_filter;
    downsample_filter.setLeafSize(0.1,0.1,0.1);
    downsample_filter.setInputCloud(map);
    pcl::PointCloud<Point3I>::Ptr filtered_map(new pcl::PointCloud<Point3I>());
    downsample_filter.filter(*filtered_map);

    filtered_map->width = filtered_map->size();
    filtered_map->height = 1;
    filtered_map->is_dense = false;

    pcl::io::savePCDFileBinary(mapfile, *filtered_map);

    graph_handler_->optimize(configuration_.optimizer_iterations_);
    graph_handler_->save(graphfile);

    return true;
}

void BackendHandler::prepare_data_for_visualization() {
    pcl::PointCloud<Point3I>::Ptr map(new pcl::PointCloud<Point3I>());
    std::vector<EigIsometry3d> poses;

    // prepare the poses and find the bounding box of the trajectory
    double min_x = 0, max_x = 0;
    double min_y = 0, max_y = 0;
    //std::cout << "BBOX: " << occmap_bbox_[0] << " " << occmap_bbox_[1] << " " << occmap_bbox_[2] << " " << occmap_bbox_[1] << std::endl;
    for(const KeyframeLaser3D::Ptr& keyframe : keyframes_) {
        EigIsometry3d pose = keyframe->graph_node_->estimate();
        poses.push_back(pose);

        double x = pose.translation().x();
        double y = pose.translation().y();
        if(x > max_x) max_x = x;
        else if(x < min_x) min_x = x;
        if(y > max_y) max_y = y;
        else if(y < min_y) min_y = y;
    }

    // check if occupancy map should be increased
    bool is_occmap_enlarged_ = false;
    if(min_x - 100 <= occmap_bbox_[0]) {
        occmap_width_ += configuration_.occmap_width_ / 2;
        occmap_bbox_[0] -= configuration_.occmap_resolution_ * static_cast<float>(configuration_.occmap_width_ / 2.0);
        is_occmap_enlarged_ = true;
    }

    if(max_x + 100 >= occmap_bbox_[1]) {
        occmap_width_ += configuration_.occmap_width_ / 2;
        occmap_bbox_[1] += configuration_.occmap_resolution_ * static_cast<float>(configuration_.occmap_width_ / 2.0);
        is_occmap_enlarged_ = true;
    }

    if(min_y - 100 <= occmap_bbox_[2]) {
        occmap_height_ += configuration_.occmap_height_ / 2;
        occmap_bbox_[2] -= configuration_.occmap_resolution_ * static_cast<float>(configuration_.occmap_height_ / 2.0);
        is_occmap_enlarged_ = true;
    }

    if(max_y + 100 >= occmap_bbox_[3]) {
        occmap_height_ += configuration_.occmap_height_ / 2;
        occmap_bbox_[3] += configuration_.occmap_resolution_ * static_cast<float>(configuration_.occmap_height_ / 2.0);
        is_occmap_enlarged_ = true;
    }

    // re compute the map in case of enlargement or pose graph update
    if(update_occmap_ || is_occmap_enlarged_) {
        occupancygrid_ = std::make_shared<OccupancyGrid>();
        occupancygrid_->width_ = occmap_width_;
        occupancygrid_->height_ = occmap_height_;
        occupancygrid_->resolution_ = configuration_.occmap_resolution_;
        occupancygrid_->data_.assign(occmap_width_ * occmap_height_, -1);
        occupancygrid_->initial_pose_ = EigIsometry3d::Identity();
        occupancygrid_->initial_pose_.translation() = EigVector3d(occmap_bbox_[0], occmap_bbox_[2], 0);

        for(int i = 0; i < keyframes_.size() - configuration_.maximum_unoptimized_keyframes_; i++) {
            EigIsometry3d pose = keyframes_[i]->graph_node_->estimate();
            poses.push_back(pose);
            EigMatrix4d posem = pose.matrix();
            pcl::PointCloud<Point3I>::Ptr transformed_pointcloud(new pcl::PointCloud<Point3I>());
            pcl::transformPointCloud(*(keyframes_[i]->pointcloud_2d_), *transformed_pointcloud, posem);

            // ray tracing
            EigVector2d origin = posem.block<2,1>(0,3);
            int r_pos = static_cast<int>((origin.y() - occmap_bbox_[2]) / configuration_.occmap_resolution_);
            int c_pos = static_cast<int>((origin.x() - occmap_bbox_[0]) / configuration_.occmap_resolution_);
            double orig_pos_x = static_cast<float>(c_pos) * configuration_.occmap_resolution_ + occmap_bbox_[0] + configuration_.occmap_resolution_ / 2;
            double orig_pos_y = static_cast<float>(r_pos) * configuration_.occmap_resolution_ + occmap_bbox_[2] + configuration_.occmap_resolution_ / 2;

            for(int j = 0; j < transformed_pointcloud->size(); j++) {
                EigVector2d pt(transformed_pointcloud->points[j].x, transformed_pointcloud->points[j].y);

                if(pt.x() >= occmap_bbox_[1] || pt.x() <= occmap_bbox_[0] || pt.y() >= occmap_bbox_[3] || pt.y() <= occmap_bbox_[2])
                    continue;

                int r_pt = static_cast<int>((pt.y() - occmap_bbox_[2]) / configuration_.occmap_resolution_);
                int c_pt = static_cast<int>((pt.x() - occmap_bbox_[0]) / configuration_.occmap_resolution_);
                double orig_pt_x = static_cast<float>(c_pt) * configuration_.occmap_resolution_ + occmap_bbox_[0] + configuration_.occmap_resolution_ / 2;
                double orig_pt_y = static_cast<float>(r_pt) * configuration_.occmap_resolution_ + occmap_bbox_[2] + configuration_.occmap_resolution_ / 2;

                occupancygrid_->data_[occmap_width_ * r_pt + c_pt] = 100;
                supercover_line(orig_pos_x, orig_pos_y, orig_pt_x, orig_pt_y);
            }
        }
    }

    for(int i = keyframes_.size() - 1 - configuration_.maximum_unoptimized_keyframes_; i < keyframes_.size(); i++) {
        EigIsometry3d pose = keyframes_[i]->graph_node_->estimate();
        EigMatrix4d posem = pose.matrix();
        pcl::PointCloud<Point3I>::Ptr transformed_pointcloud(new pcl::PointCloud<Point3I>());
        pcl::PointCloud<Point3I>::Ptr transformed_pointcloud3d(new pcl::PointCloud<Point3I>());
        pcl::transformPointCloud(*(keyframes_[i]->pointcloud_2d_), *transformed_pointcloud, posem);
        pcl::transformPointCloud(*(keyframes_[i]->pointcloud_), *transformed_pointcloud3d, posem);

        // ray tracing
        EigVector2d origin = posem.block<2, 1>(0, 3);
        int r_pos = static_cast<int>((origin.y() - occmap_bbox_[2]) / configuration_.occmap_resolution_);
        int c_pos = static_cast<int>((origin.x() - occmap_bbox_[0]) / configuration_.occmap_resolution_);
        double orig_pos_x = static_cast<float>(c_pos) * configuration_.occmap_resolution_ + occmap_bbox_[0] + configuration_.occmap_resolution_ / 2;
        double orig_pos_y = static_cast<float>(r_pos) * configuration_.occmap_resolution_ + occmap_bbox_[2] + configuration_.occmap_resolution_ / 2;

        for (int j = 0; j < transformed_pointcloud->size(); j++) {
            EigVector2d pt(transformed_pointcloud->points[j].x, transformed_pointcloud->points[j].y);

            if(pt.x() >= occmap_bbox_[1] || pt.x() <= occmap_bbox_[0] || pt.y() >= occmap_bbox_[3] || pt.y() <= occmap_bbox_[2])
                continue;

            int r_pt = static_cast<int>((pt.y() - occmap_bbox_[2]) / configuration_.occmap_resolution_);
            int c_pt = static_cast<int>((pt.x() - occmap_bbox_[0]) / configuration_.occmap_resolution_);
            double orig_pt_x = static_cast<float>(c_pt) * configuration_.occmap_resolution_ + occmap_bbox_[0] + configuration_.occmap_resolution_ / 2;
            double orig_pt_y = static_cast<float>(r_pt) * configuration_.occmap_resolution_ + occmap_bbox_[2] + configuration_.occmap_resolution_ / 2;

            occupancygrid_->data_[occmap_width_ * r_pt + c_pt] = 100;
            supercover_line(orig_pos_x, orig_pos_y, orig_pt_x, orig_pt_y);
        }

        // point cloud map
        *map += *transformed_pointcloud;
    }

    /*for(const KeyframeLaser3D::Ptr& keyframe : keyframes_) {
        EigIsometry3d pose = keyframe->graph_node_->estimate();
        EigMatrix4d posem = pose.matrix();
        pcl::PointCloud<Point3I>::Ptr transformed_pointcloud(new pcl::PointCloud<Point3I>());
        pcl::PointCloud<Point3I>::Ptr transformed_pointcloud3d(new pcl::PointCloud<Point3I>());
        pcl::transformPointCloud(*(keyframe->pointcloud_2d_), *transformed_pointcloud, posem);
        pcl::transformPointCloud(*(keyframe->pointcloud_), *transformed_pointcloud3d, posem);

        // ray tracing
        EigVector2d origin = posem.block<2,1>(0,3);
        int r_pos = (origin.y() + 125) / 0.25;
        int c_pos = (origin.x() + 125) / 0.25;
        double orig_pos_x = c_pos * 0.25 - 125 + 0.25/2;
        double orig_pos_y = r_pos * 0.25 - 125 + 0.25/2;
        for(int i = 0; i < transformed_pointcloud->size(); i++) {
            EigVector2d pt(transformed_pointcloud->points[i].x, transformed_pointcloud->points[i].y);

            //if(pt.x() > 80 || pt.x() < -80 || pt.y() > 80 || pt.y() < -80)
                //continue;

            int r_pt = (pt.y() + 125) / 0.25;
            int c_pt = (pt.x() + 125) / 0.25;
            double orig_pt_x = c_pt * 0.25 - 125 + 0.25/2;
            double orig_pt_y = r_pt * 0.25 - 125 + 0.25/2;

            occupancygrid_.data_[1000 * r_pt + c_pt] = 100;
            supercover_line(orig_pos_x, orig_pos_y, orig_pt_x, orig_pt_y);
        }

        // point cloud map
        *map += *transformed_pointcloud;
    }*/

    //std::cout << "BACKEND: " << occmap_width_ << " " << occmap_height_ << std::endl;
    notify_slam_output_observers(map, poses);

    /*if(!poses.empty() && !map->empty()) {
        pcl::VoxelGrid<Point3I> filter;
        filter.setLeafSize(0., 0.4, 0.4);
        filter.setInputCloud(map);
        filter.filter(*map);
        notify_slam_output_observers(map, poses);
    }*/

    /*if(!keyframes_.empty()) {
        EigIsometry3d pose = keyframes_.back()->graph_node_->estimate();
        EigMatrix4d posem = pose.matrix();
        pcl::PointCloud<Point3I>::Ptr transformed_pointcloud(new pcl::PointCloud<Point3I>());
        pcl::transformPointCloud(*(keyframes_.back()->pointcloud_), *transformed_pointcloud, posem);
        *map += *transformed_pointcloud;
        notify_slam_output_observers(map, poses);
    }*/
}

void BackendHandler::supercover_line(double x0, double y0, double x1, double y1) {
    double dx = x1 - x0;
    double dy = y1 - y0;

    double nx = abs(dx);
    double ny = abs(dy);

    double step_x = dx > 0 ? configuration_.occmap_resolution_ : -configuration_.occmap_resolution_;
    double step_y = dy > 0 ? configuration_.occmap_resolution_ : -configuration_.occmap_resolution_;

    double ix = 0, iy = 0;

    while(ix < nx || iy < ny) {
        double dec = (1 + (2 / configuration_.occmap_resolution_) * ix) * ny - (1 + (2 / configuration_.occmap_resolution_) * iy) * nx;
        if(dec == 0) {
            // next step is diagonal
            x0 += step_x;
            y0 += step_y;
            ix += configuration_.occmap_resolution_;
            iy += configuration_.occmap_resolution_;
        } else if(dec < 0) {
            // next step is horizontal
            x0 += step_x;
            ix += configuration_.occmap_resolution_;
        } else {
            // next step is vertical
            y0 += step_y;
            iy += configuration_.occmap_resolution_;
        }

        int r_pt = (y0 - occmap_bbox_[2]) / configuration_.occmap_resolution_;
        int c_pt = (x0 - occmap_bbox_[0]) / configuration_.occmap_resolution_;
        if(occupancygrid_->data_[occmap_width_ * r_pt + c_pt] == 100) break;
        occupancygrid_->data_[occmap_width_ * r_pt + c_pt] = 0;
    }
}
