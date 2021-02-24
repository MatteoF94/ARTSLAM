/** @file backend.cpp
 * @brief Definition of class Backend
 * @author Matteo Frosi
 */

#include <backend.h>
#include <loop.h>
#include <cmath>
#include <chrono>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <bin_io.h>
#include <boost/format.hpp>

/**
 * @brief Class constructor.
 */
Backend::Backend() {
    BackendConfig default_config;
    configure_backend(&default_config);

    this->trans_odom2map = Eigen::Matrix4f::Identity();
    this->anchor_node = nullptr;
    this->anchor_edge = nullptr;
    this->floor_plane_node = nullptr;
    this->insertion_dqueue = new DispatchQueue("backend insertion queue", 1);
    this->optimization_dqueue = new DispatchQueue("optimization queue", 1);
}

Backend::Backend(const BackendConfig* config_ptr) {
    configure_backend(config_ptr);

    this->anchor_node = nullptr;
    this->anchor_edge = nullptr;
    this->floor_plane_node = nullptr;
    this->trans_odom2map = Eigen::Matrix4f::Identity();
    this->insertion_dqueue = new DispatchQueue("backend insertion queue", 1);
    this->optimization_dqueue = new DispatchQueue("optimization queue", 1);
}

/**
 * @brief Class destructor.
 */
Backend::~Backend() {
    this->graph_handler->save("/home/matteo/Desktop/grafo_test.g2o");
    delete this->insertion_dqueue;
    delete this->optimization_dqueue;
}

void Backend::configure_backend(const BackendConfig *config_ptr) {
    this->max_unopt_keyframes = config_ptr->max_unopt_keyframes;
    this->optimizer_iters = config_ptr->optimizer_iters;

    this->use_anchor_node = config_ptr->use_anchor_node;
    this->anchor_node_stddev = config_ptr->anchor_node_stddev;
    this->fix_first_node_adaptive = config_ptr->fix_first_node_adaptive;
    this->fix_first_keyframe_node = config_ptr->fix_first_keyframe_node;
    this->odometry_edge_robust_kernel = config_ptr->odometry_edge_robust_kernel;
    this->odometry_edge_robust_kernel_size = config_ptr->odometry_edge_robust_kernel_size;

    this->floor_edge_stddev = config_ptr->floor_edge_stddev;
    this->floor_edge_robust_kernel = config_ptr->floor_edge_robust_kernel;
    this->floor_edge_robust_kernel_size = config_ptr->odometry_edge_robust_kernel_size;

    this->is_imu_accel_enabled = config_ptr->is_imu_accel_enabled;
    this->is_imu_orien_enabled = config_ptr->is_imu_orien_enabled;
    this->imu_to_velo_rot = config_ptr->imu_to_velo_rot;
    this->imu_accel_edge_stddev = config_ptr->imu_accel_edge_stddev;
    this->imu_orien_edge_stddev = config_ptr->imu_orien_edge_stddev;
    this->imu_accel_edge_robust_kernel = config_ptr->imu_accel_edge_robust_kernel;
    this->imu_accel_edge_robust_kernel_size = config_ptr->imu_accel_edge_robust_kernel_size;
    this->imu_orien_edge_robust_kernel = config_ptr->imu_orien_edge_robust_kernel;
    this->imu_orien_edge_robust_kernel_size = config_ptr->imu_orien_edge_robust_kernel_size;

    this->is_gps_enabled = config_ptr->is_gps_enabled;
    this->gps_to_velo_trans = config_ptr->gps_to_velo_trans;
    this->gps_xy_edge_stddev = config_ptr->gps_xy_edge_stddev;
    this->gps_z_edge_stddev = config_ptr->gps_z_edge_stddev;
    this->gps_edge_robust_kernel = config_ptr->gps_edge_robust_kernel;
    this->gps_edge_robust_kernel_size = config_ptr->gps_edge_robust_kernel_size;

    this->loop_edge_robust_kernel = config_ptr->loop_edge_robust_kernel;
    this->loop_edge_robust_kernel_size = config_ptr->loop_edge_robust_kernel_size;
}

/**
 * @brief Sets the pose graph handler.
 */
void Backend::set_graph_handler(GraphHandler *const graph_handler_ptr) {
    this->graph_handler = graph_handler_ptr;
}

/**
 * @brief Sets the information matrix calculator.
 */
void Backend::set_info_matrix_calculator(InfoMatrixCalculator *const info_matrix_calculator_ptr) {
    this->info_matrix_calculator = info_matrix_calculator_ptr;
}

/**
 * @brief Sets the loop detector.
 */
void Backend::set_loop_detector(LoopDetector *const loop_detector_ptr) {
    this->loop_detector = loop_detector_ptr;
}

/* ---------------------------------------- */
/* Signals that a new keyframe has arrived. */
/* ---------------------------------------- */
void Backend::update(const Keyframe::Ptr& keyframe) {
    dispatch_keyframe_insertion(keyframe);
}

/* --------------------------------------------------------- */
/* Signals that a new set of floor coefficients has arrived. */
/* --------------------------------------------------------- */
void Backend::update(const FloorCoeffsMSG::ConstPtr& floor_coeffs_msg_constptr) {
    dispatch_floor_coeffs_msg_insertion(floor_coeffs_msg_constptr);
}

/* -------------------------------------- */
/* Signals that new IMU data has arrived. */
/* -------------------------------------- */
void Backend::update(const ImuMSG::ConstPtr &imu_msg_constptr) {
    dispatch_imu_msg_insertion(imu_msg_constptr);
}

/* -------------------------------------- */
/* Signals that new GPS data has arrived. */
/* -------------------------------------- */
void Backend::update(const GeoPointStampedMSG::ConstPtr& gps_msg_constptr) {
    dispatch_gps_msg_insertion(gps_msg_constptr);
}

void Backend::optimize() {
    std::lock_guard<std::mutex> lock(this->backend_thread_mutex);

    flush_keyframe_queue();
    flush_floor_coeffs_msg_queue();
    flush_imu_msg_queue();
    flush_gps_msg_queue();

    auto start = std::chrono::high_resolution_clock::now();
    loop_detection();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "OPTIMIZATION: " << duration.count() << "\n";

    // let the first node move freely around the origin
    if(this->anchor_node && this->fix_first_node_adaptive) {
        Eigen::Isometry3d anchor_target = static_cast<g2o::VertexSE3*>(this->anchor_edge->vertices()[1])->estimate();
        this->anchor_node->setEstimate(anchor_target);
    }

    // optimize graph
    this->graph_handler->optimize(this->optimizer_iters);
}

void Backend::loop_detection() {
    std::vector<Loop::Ptr> loops = this->loop_detector->detect(this->keyframes, this->new_keyframes);
    std::ofstream outp;
    outp.open("/home/matteo/Desktop/loops.txt", std::ios_base::app);

    for(const Loop::Ptr& loop : loops) {
        outp << loop->keyframe_1->cloud->header.seq << " - " << loop->keyframe_2->cloud->header.seq << std::endl;
        outp << loop->relative_pose << std::endl << std::endl;

        Eigen::Isometry3d relative_pose(loop->relative_pose.cast<double>());
        Eigen::MatrixXd info_matrix = this->info_matrix_calculator->calculate_info_matrix(loop->keyframe_1->cloud, loop->keyframe_2->cloud, relative_pose);

        g2o::EdgeSE3* edge = this->graph_handler->add_se3_edge(loop->keyframe_1->node, loop->keyframe_2->node, relative_pose, info_matrix);
        this->graph_handler->add_robust_kernel(edge, this->loop_edge_robust_kernel, this->loop_edge_robust_kernel_size);
    }

    std::copy(this->new_keyframes.begin(), this->new_keyframes.end(), std::back_inserter(this->keyframes));
    this->new_keyframes.clear();
}

/* --------------------------------------- */
/* Adds keyframes poses to the pose graph. */
/* --------------------------------------- */
void Backend::flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(this->keyframe_queue_mutex);

    if(this->keyframe_queue.empty()) {
        return;
    }

    // odom2map transformation
    this->trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(this->trans_odom2map.cast<double>());
    this->trans_odom2map_mutex.unlock();

    for(int i = 0; i < this->keyframe_queue.size(); i++) {
        Keyframe::Ptr keyframe = this->keyframe_queue[i];
        this->new_keyframes.emplace_back(keyframe);

        // add pose node
        Eigen::Isometry3d odom = odom2map * keyframe->odom;
        keyframe->node = this->graph_handler->add_se3_node(odom);
        this->keyframe_hash[keyframe->timestamp] = keyframe;

        if(this->keyframes.empty() && this->new_keyframes.size() == 1) {
            if(this->use_anchor_node) {
                Eigen::MatrixXd info_matrix = Eigen::MatrixXd::Identity(6,6);
                for(int j = 0; j < 6; j++) {
                    double stddev = 1.0;
                    if(this->anchor_node_stddev.size() == 6)
                        stddev = this->anchor_node_stddev[j];
                    info_matrix(j,j) = 1.0 / stddev;
                }

                this->anchor_node = this->graph_handler->add_se3_node(Eigen::Isometry3d::Identity());
                this->anchor_node->setFixed(true);
                this->anchor_edge = this->graph_handler->add_se3_edge(this->anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), info_matrix);
            }

            if(this->fix_first_keyframe_node) {
                keyframe->node->setFixed(true);
            }
        }

        if(i == 0 && this->keyframes.empty()) {
            continue;
        }

        const Keyframe::Ptr prev_keyframe = i == 0 ? this->keyframes.back() : this->keyframe_queue[i - 1];

        Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
        Eigen::MatrixXd info_matrix = this->info_matrix_calculator->calculate_info_matrix(keyframe->fitness_score);
        //Eigen::MatrixXd info_matrix = this->info_matrix_calculator->calculate_info_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);
        g2o::EdgeSE3* edge = this->graph_handler->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, info_matrix);
        this->graph_handler->add_robust_kernel(edge, this->odometry_edge_robust_kernel, this->odometry_edge_robust_kernel_size);
    }

    this->keyframe_queue.clear();
}

/* ------------------------------------------------------------------------------------- */
/* Associates floor coefficients to the keyframes, adding constraints in the pose graph. */
/* ------------------------------------------------------------------------------------- */
void Backend::flush_floor_coeffs_msg_queue() {
    std::lock_guard<std::mutex> lock(this->floor_coeffs_msg_queue_mutex);

    // should never happen in the current implementation, maybe some one would like to parallelize the flush operations...
    if(this->keyframes.empty() || this->floor_coeffs_msg_queue.empty()) {
        return;
    }

    if(!this->floor_plane_node) {
        this->floor_plane_node = this->graph_handler->add_plane_node(Eigen::Vector4d(0.0,0.0,1.0,0.0));
        this->floor_plane_node->setFixed(true);
    }

    const uint64_t latest_keyframe_timestamp = this->keyframes.back()->timestamp;

    for(const FloorCoeffsMSG::ConstPtr& floor_coeffs_msg : this->floor_coeffs_msg_queue) {
        // under the assumption that floor coeffs are ordered
        if(floor_coeffs_msg->header.timestamp > latest_keyframe_timestamp) {
            break;
        }

        auto found = this->keyframe_hash.find(floor_coeffs_msg->header.timestamp);
        if(found == this->keyframe_hash.end()) {
            continue;
        }

        Keyframe::Ptr keyframe = found->second;
        Eigen::Vector4d coeffs(floor_coeffs_msg->floor_coeffs);
        Eigen::Matrix3d info_matrix = Eigen::Matrix3d::Identity() / this->floor_edge_stddev;
        g2o::EdgeSE3Plane* edge = this->graph_handler->add_se3_plane_edge(keyframe->node, this->floor_plane_node, coeffs, info_matrix);
        this->graph_handler->add_robust_kernel(edge, this->floor_edge_robust_kernel, this->floor_edge_robust_kernel_size);

        keyframe->floor_coeffs = coeffs;
    }

    auto remove_location = std::upper_bound(this->floor_coeffs_msg_queue.begin(),
                                            this->floor_coeffs_msg_queue.end(),
                                             latest_keyframe_timestamp,
                                             [=](const uint64_t& timestamp, const FloorCoeffsMSG::ConstPtr& coeffs) {return timestamp < coeffs->header.timestamp;});
    this->floor_coeffs_msg_queue.erase(this->floor_coeffs_msg_queue.begin(), remove_location);
}

/* -------------------------------------------------------------------------- */
/* Associates IMU data to the keyframes, adding constraints in the pose graph. */
/* -------------------------------------------------------------------------- */
void Backend::flush_imu_msg_queue() {
   std::lock_guard<std::mutex> lock(this->imu_msg_queue_mutex);

    if(!this->is_imu_accel_enabled && !this->is_imu_orien_enabled) {
        return;
    }

   if(this->keyframes.empty() || this->imu_msg_queue.empty()) {
       return;
   }

   auto imu_cursor = this->imu_msg_queue.begin();

   for(auto& keyframe : this->keyframes) {
       // if the keyframe timestamp is greater than the timestamp of the last received IMU data, stop the data association
       if(keyframe->timestamp > this->imu_msg_queue.back()->header.timestamp) {
           break;
       }

       if(keyframe->timestamp > (*imu_cursor)->header.timestamp || keyframe->acceleration) {
           continue;
       }

       // find the closest IMU data to the keyframe
       auto closest_imu = imu_cursor;
       for(auto imu = imu_cursor; imu != this->imu_msg_queue.end(); imu++) {
           uint64_t dt = ((*closest_imu)->header.timestamp > keyframe->timestamp) ? (*closest_imu)->header.timestamp - keyframe->timestamp : keyframe->timestamp - (*closest_imu)->header.timestamp;
           uint64_t dt2 = ((*imu)->header.timestamp - keyframe->timestamp) ? (*imu)->header.timestamp - keyframe->timestamp : keyframe->timestamp - (*imu)->header.timestamp;
           if(dt < dt2) {
               break;
           }

           closest_imu = imu;
       }

       imu_cursor = closest_imu;
       uint64_t dt = ((*closest_imu)->header.timestamp > keyframe->timestamp) ? (*closest_imu)->header.timestamp - keyframe->timestamp : keyframe->timestamp - (*closest_imu)->header.timestamp;
       if(dt > 0.2*1e9) {
           continue;
       }

       if((*closest_imu)->has_linear_acceleration) {
           const auto &imu_accel = (*closest_imu)->linear_acceleration;
           keyframe->acceleration = this->imu_to_velo_rot * imu_accel;
           if (this->is_imu_accel_enabled) {
               Eigen::MatrixXd info_matrix = Eigen::MatrixXd::Identity(3, 3) / this->imu_accel_edge_stddev;
               g2o::OptimizableGraph::Edge *edge = this->graph_handler->add_se3_prior_vec_edge(keyframe->node,
                                                                                               -Eigen::Vector3d::UnitZ(),
                                                                                               *keyframe->acceleration,
                                                                                               info_matrix);
               this->graph_handler->add_robust_kernel(edge, this->imu_accel_edge_robust_kernel, this->imu_accel_edge_robust_kernel_size);
           }
       }

       if((*closest_imu)->has_orientation) {
           const auto &imu_orien = (*closest_imu)->orientation;
           keyframe->orientation = rot_to_quat(this->imu_to_velo_rot) * imu_orien;
           if (keyframe->orientation->w() < 0.0) {
               keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
           }
           if (this->is_imu_orien_enabled) {
               Eigen::MatrixXd info_matrix = Eigen::MatrixXd::Identity(3, 3) / this->imu_orien_edge_stddev;
               auto *edge = this->graph_handler->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, info_matrix);
               this->graph_handler->add_robust_kernel(edge, this->imu_orien_edge_robust_kernel, this->imu_orien_edge_robust_kernel_size);
           }
       }
   }

   auto remove_loc = std::upper_bound(this->imu_msg_queue.begin(), this->imu_msg_queue.end(), this->keyframes.back()->timestamp,
                                      [=](uint64_t timestamp, const ImuMSG::ConstPtr& imu){return timestamp < imu->header.timestamp;});
   this->imu_msg_queue.erase(this->imu_msg_queue.begin(), remove_loc);
}

/* -------------------------------------------------------------------------- */
/* Associates GPS data to the keyframes, adding constraints in the pose graph. */
/* -------------------------------------------------------------------------- */
void Backend::flush_gps_msg_queue() {
    std::lock_guard<std::mutex> lock(this->gps_msg_queue_mutex);

    if(!this->is_gps_enabled) {
        return;
    }

    if(this->keyframes.empty() || this->gps_msg_queue.empty()) {
        return;
    }

    auto gps_cursor = this->gps_msg_queue.begin();

    for(auto& keyframe : this->keyframes) {
        // if the keyframe timestamp is greater than the timestamp of the last received GPS data, stop the data association
        if (keyframe->timestamp > this->gps_msg_queue.back()->header.timestamp) {
            break;
        }

        if (keyframe->timestamp > (*gps_cursor)->header.timestamp || keyframe->acceleration) {
            continue;
        }

        // find the closest IMU data to the keyframe
        auto closest_gps = gps_cursor;
        for (auto gps = gps_cursor; gps != this->gps_msg_queue.end(); gps++) {
            uint64_t dt = ((*closest_gps)->header.timestamp > keyframe->timestamp) ? (*closest_gps)->header.timestamp -
                                                                                     keyframe->timestamp :
                          keyframe->timestamp - (*closest_gps)->header.timestamp;
            uint64_t dt2 = ((*gps)->header.timestamp - keyframe->timestamp) ? (*gps)->header.timestamp -
                                                                              keyframe->timestamp :
                           keyframe->timestamp - (*gps)->header.timestamp;
            if (dt < dt2) {
                break;
            }

            closest_gps = gps;
        }

        gps_cursor = closest_gps;
        uint64_t dt = ((*closest_gps)->header.timestamp > keyframe->timestamp) ? (*closest_gps)->header.timestamp -
                                                                                 keyframe->timestamp :
                      keyframe->timestamp - (*closest_gps)->header.timestamp;
        if (dt > 0.2 * 1e9) {
            continue;
        }

        int zone;
        bool nortph;
        double easting, northing;
        GeographicLib::UTMUPS::Forward((*gps_cursor)->lat, (*gps_cursor)->lon, zone, nortph, easting, northing);
        Eigen::Vector3d xyz(easting, northing, (*gps_cursor)->alt);
        xyz += this->gps_to_velo_trans;

        if(this->first_utm) {
            this->first_utm = xyz;
        }
        xyz -= *(this->first_utm);

        keyframe->utm_coords = xyz;

        g2o::OptimizableGraph::Edge* edge;
        if(std::isnan(xyz.z())) {
            Eigen::Matrix2d info_matrix = Eigen::Matrix2d::Identity() / this->gps_xy_edge_stddev;
            edge = this->graph_handler->add_se3_prior_xy_edge(keyframe->node, xyz.head(2), info_matrix);
        } else {
            Eigen::Matrix3d info_matrix = Eigen::Matrix3d::Identity();
            info_matrix.block<2,2>(0,0) /= this->gps_xy_edge_stddev;
            info_matrix(2,2) /= this->gps_z_edge_stddev;
            edge = this->graph_handler->add_se3_prior_xyz_edge(keyframe->node, xyz, info_matrix);
        }
        this->graph_handler->add_robust_kernel(edge, this->gps_edge_robust_kernel, this->gps_edge_robust_kernel_size);
    }

    auto remove_loc = std::upper_bound(this->gps_msg_queue.begin(), this->gps_msg_queue.end(), this->keyframes.back()->timestamp,
                                       [=](uint64_t timestamp, const GeoPointStampedMSG::ConstPtr& gps){return timestamp < gps->header.timestamp;});
    this->gps_msg_queue.erase(this->gps_msg_queue.begin(), remove_loc);
}

/* ------------------------------- */
/* Adds a keyframe to the backend. */
/* ------------------------------- */
void Backend::insert_keyframe(const Keyframe::Ptr& keyframe) {
    std::lock_guard<std::mutex> lock(this->keyframe_queue_mutex);
    this->keyframe_queue.emplace_back(keyframe);

    if(this->keyframe_queue.size() >= this->max_unopt_keyframes) {
        this->optimization_dqueue->dispatch([this]{this->optimize();});
    }
}

/* ------------------------------------------------ */
/* Adds computed floor coefficients to the backend. */
/* ------------------------------------------------ */
void Backend::insert_floor_coeffs_msg(const FloorCoeffsMSG::ConstPtr& floor_coeffs_msg_constptr) {
    std::lock_guard<std::mutex> lock(this->floor_coeffs_msg_queue_mutex);
    this->floor_coeffs_msg_queue.emplace_back(floor_coeffs_msg_constptr);
}

/* ----------------------------- */
/* Adds IMU data to the backend. */
/* ----------------------------- */
void Backend::insert_imu_msg(const ImuMSG::ConstPtr &imu_msg_constptr) {
    std::lock_guard<std::mutex> lock(this->imu_msg_queue_mutex);

    if(!this->is_imu_accel_enabled && !this->is_imu_orien_enabled) {
        return;
    }

    this->imu_msg_queue.emplace_back(imu_msg_constptr);
}

/* ----------------------------- */
/* Adds GPS data to the backend. */
/* ----------------------------- */
void Backend::insert_gps_msg(const GeoPointStampedMSG::ConstPtr& gps_msg_constptr) {
    std::lock_guard<std::mutex> lock(this->gps_msg_queue_mutex);

    if(!this->is_gps_enabled) {
        return;
    }

    this->gps_msg_queue.emplace_back(gps_msg_constptr);
}

/* --------------------------------------------------------------------------------------------------- */
/* Tells the backend that a keyframe has arrived and it is waiting to be integrated in the pose graph. */
/* --------------------------------------------------------------------------------------------------- */
void Backend::dispatch_keyframe_insertion(const Keyframe::Ptr& keyframe) {
    this->insertion_dqueue->dispatch([this, keyframe]{insert_keyframe(keyframe);});
}

/* -------------------------------------------------------------------------------------------------------------------- */
/* Tells the backend that a set of floor coefficients has arrived and it is waiting to be integrated in the pose graph. */
/* -------------------------------------------------------------------------------------------------------------------- */
void Backend::dispatch_floor_coeffs_msg_insertion(const FloorCoeffsMSG::ConstPtr& floor_coeffs_msg_constptr) {
    this->insertion_dqueue->dispatch([this, floor_coeffs_msg_constptr]{insert_floor_coeffs_msg(floor_coeffs_msg_constptr);});
}

/* ----------------------------------------------------------------------------------------------------- */
/* Tells the backend that new IMU data has arrived and it is waiting to be integrated in the pose graph. */
/* ----------------------------------------------------------------------------------------------------- */
void Backend::dispatch_imu_msg_insertion(const ImuMSG::ConstPtr& imu_msg_constptr) {
    this->insertion_dqueue->dispatch([this, imu_msg_constptr]{insert_imu_msg(imu_msg_constptr);});
}

/* ----------------------------------------------------------------------------------------------------- */
/* Tells the backend that new GPS data has arrived and it is waiting to be integrated in the pose graph. */
/* ----------------------------------------------------------------------------------------------------- */
void Backend::dispatch_gps_msg_insertion(const GeoPointStampedMSG::ConstPtr& gps_msg_constptr) {
    this->insertion_dqueue->dispatch([this, gps_msg_constptr]{insert_gps_msg(gps_msg_constptr);});
}

/* -------------------------------------------------- */
/* Converts a rotation matrix to its quaternion form. */
/* -------------------------------------------------- */
Eigen::Quaterniond Backend::rot_to_quat(const Eigen::Matrix3d& rot) {
    double r11 = rot(0,0);
    double r12 = rot(0,1);
    double r13 = rot(0,2);
    double r21 = rot(1,0);
    double r22 = rot(1,1);
    double r23 = rot(1,2);
    double r31 = rot(2,0);
    double r32 = rot(2,1);
    double r33 = rot(2,2);

    double q0 = (r11 + r22 + r33 + 1.0) / 4.0;
    double q1 = (r11 - r22 - r33 + 1.0) / 4.0;
    double q2 = (-r11 + r22 - r33 + 1.0) / 4.0;
    double q3 = (-r11 -r22 +r33 +1.0) / 4.0;

    if(q0 < 0.0) q0 = 0.0;
    if(q1 < 0.0) q1 = 0.0;
    if(q2 < 0.0) q2 = 0.0;
    if(q3 < 0.0) q3 = 0.0;

    q0 = std::sqrt(q0);
    q1 = std::sqrt(q1);
    q2 = std::sqrt(q2);
    q3 = std::sqrt(q3);

    if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
        q0 *= 1.0;
        q1 *= r32 >= r23 ? 1.0 : -1.0;
        q2 *= r13 >= r31 ? 1.0 : -1.0;
        q3 *= r21 >= r12 ? 1.0 : -1.0;
    } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
        q0 *= r32 >= r23 ? 1.0 : -1.0;
        q1 *= 1.0;
        q2 *= (r21 + r12) >= 0 ? 1.0 : -1.0;
        q3 *= (r13 + r31) >= 0 ? 1.0 : -1.0;
    } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
        q0 *= r13 >= r31 ? 1.0 : -1.0;
        q1 *= (r21 + r12) >= 0 ? 1.0 : -1.0;
        q2 *= 1.0;
        q3 *= (r32 + r23) >= 0 ? 1.0 : -1.0;
    } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
        q0 *= r21 >= r12 ? 1.0 : -1.0;
        q1 *= (r13 + r31) >= 0 ? 1.0 : -1.0;
        q2 *= (r32 + r23) >= 0 ? 1.0 : -1.0;
        q3 *= 1.0;
    }

    double norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    Eigen::Quaterniond quaternion(q0,q1,q2,q3);
    return quaternion;
}

void Backend::save_data(const std::string& path) {
    std::lock_guard<std::mutex> lock(this->backend_thread_mutex);

    if(!boost::filesystem::is_directory(path)) {
        boost::filesystem::create_directory(path);
    }

    this->graph_handler->save(path + "/graph.g2o");
    for(int i = 0; i < this->keyframes.size(); i++) {
        std::stringstream sst;
        sst << boost::format("%s/%06d") % path % i;

        this->keyframes[i]->save_to_dir(sst.str());
    }

    if(this->first_utm) {
        std::ofstream first_utm_ofs(path + "/first_utm");
        first_utm_ofs << *first_utm << std::endl;
    }

    std::ofstream ofs(path + "/special_nodes.csv");
    ofs << "anchor_node " << (this->anchor_node == nullptr ? -1 : this->anchor_node->id()) << std::endl;
    ofs << "anchor_edge " << (this->anchor_edge == nullptr ? -1 : this->anchor_edge->id()) << std::endl;
    ofs << "floor_node " << (this->floor_plane_node == nullptr ? -1 : this->floor_plane_node->id()) << std::endl;

    std::ofstream kid(path + "/key_ids.txt");
    for(auto & keyframe : this->keyframes) {
        kid << keyframe->cloud->header.seq << std::endl;
    }
}