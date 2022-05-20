#include "configuration_parser.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace artslam::laser3d {
    std::vector<std::string> parse_slam_paths(const std::string& filename) {
        boost::property_tree::ptree root;
        std::vector<std::string> slam_paths;
        try {
            boost::property_tree::read_json(filename, root);
        } catch(std::exception &e) {
            return slam_paths;
        }

        for(boost::property_tree::ptree::value_type &path : root.get_child("slam_paths")) {
            std::string path_value = path.second.data();
            slam_paths.push_back(path_value);
        }

        return slam_paths;
    }

    Prefilterer::Configuration parse_prefilterer_configuration(const std::string& filename) {
        boost::property_tree::ptree root;
        Prefilterer::Configuration config;
        try {
            boost::property_tree::read_json(filename, root);
        } catch(std::exception &e) {
            return config;
        }

        config.downsample_method_ = root.get<std::string>("prefilterer.downsample_method");
        config.downsample_resolution_ = root.get<float>("prefilterer.downsample_resolution");
        config.sample_size_ = root.get<int>("prefilterer.sample_size");
        config.outlier_removal_method_ = root.get<std::string>("prefilterer.outlier_removal_method");
        config.statistical_mean_k_ = root.get<int>("prefilterer.statistical_mean_k");
        config.statistical_stddev_ = root.get<float>("prefilterer.statistical_stddev");
        config.radius_radius_ = root.get<float>("prefilterer.radius_radius");
        config.radius_min_neighbours_ = root.get<int>("prefilterer.radius_min_neighbours");
        config.use_distance_filter_ = root.get<bool>("prefilterer.use_distance_filter");
        config.distance_near_threshold_ = root.get<float>("prefilterer.distance_near_threshold");
        config.distance_far_threshold_ = root.get<float>("prefilterer.distance_far_threshold");
        config.scan_period_ = root.get<float>("prefilterer.scan_period");
        int i = 0;
        for(boost::property_tree::ptree::value_type &val : root.get_child("prefilterer.imu_to_lidar_rotation")) {
            config.imu_to_lidar_rotation_[i] = val.second.get_value<double>();
            i++;
        }
        config.verbose_ = root.get<bool>("prefilterer.verbose");

        return config;
    }

    Tracker::Configuration parse_tracker_configuration(const std::string& filename) {
        boost::property_tree::ptree root;
        Tracker::Configuration config;
        try {
            boost::property_tree::read_json(filename, root);
        } catch(std::exception &e) {
            return config;
        }

        config.keyframe_delta_trans_ = root.get<double>("tracker.keyframe_delta_trans");
        config.keyframe_delta_angle_ = root.get<double>("tracker.keyframe_delta_angle");
        config.keyframe_delta_time_ =  root.get<uint64_t>("tracker.keyframe_delta_time");
        config.transform_thresholding_ = root.get<bool>("tracker.transform_thresholding");
        config.max_acceptable_trans_ = root.get<double>("tracker.max_acceptable_trans");
        config.max_acceptable_angle_ = root.get<double>("tracker.max_acceptable_angle");
        config.use_height_filter_ = root.get<bool>("tracker.use_height_filter");
        config.filter_axis_ = root.get<std::string>("tracker.filter_axis");
        config.min_height_ = root.get<float>("tracker.min_height");
        config.use_prior_odometry_ = root.get<bool>("tracker.use_prior_odometry");
        config.prior_odometry_delta_trans_ = root.get<double>("tracker.prior_odometry_delta_trans");
        config.prior_odometry_delta_angle_ = root.get<double>("tracker.prior_odometry_delta_angle");
        config.max_odometry_delta_time_ = root.get<uint64_t>("tracker.max_odometry_delta_time");
        int i = 0;
        std::vector<float> orientation(4);
        for(boost::property_tree::ptree::value_type &val : root.get_child("tracker.initial_orientation")) {
            orientation[i] = val.second.get_value<float>();
            i++;
        }
        config.initial_orientation_ = EigQuaternionf(orientation[0],orientation[1], orientation[2],orientation[3]);
        i = 0;
        for(boost::property_tree::ptree::value_type &val : root.get_child("tracker.initial_translation")) {
            config.initial_translation_[i] = val.second.get_value<float>();
            i++;
        }
        config.utm_compensated_ = root.get<bool>("tracker.utm_compensated");
        config.verbose_ = root.get<bool>("tracker.verbose");

        return config;
    }

    GroundDetector::Configuration parse_ground_detector_configuration(const std::string & filename) {
        boost::property_tree::ptree root;
        GroundDetector::Configuration config;
        try {
            boost::property_tree::read_json(filename, root);
        } catch(std::exception &e) {
            return config;
        }

        config.tilt_angle_ = root.get<float>("ground_detector.tilt_angle");
        config.sensor_height_ = root.get<float>("ground_detector.sensor_height");
        config.clipping_range_ = root.get<float>("ground_detector.clipping_range");
        config.floor_points_threshold_ = root.get<int>("ground_detector.floor_points_threshold");
        config.use_normal_filtering_ = root.get<bool>("ground_detector.use_normal_filtering");
        config.normal_filtering_threshold_ = root.get<float>("ground_detector.normal_filtering_threshold");
        config.rough_ground_check_ = root.get<bool>("ground_detector.rough_ground_check");
        config.rough_ground_max_dist_ = root.get<float>("ground_detector.rough_ground_max_dist");
        config.verbose_ = root.get<bool>("ground_detector.verbose");

        return config;
    }

    Registration::Configuration parse_registration_tracker_configuration(const std::string& filename) {
        boost::property_tree::ptree root;
        Registration::Configuration config;
        try {
            boost::property_tree::read_json(filename, root);
        } catch(std::exception &e) {
            return config;
        }

        config.registration_method_name_ = root.get<std::string>("registration_tracker.registration_method_name");
        config.registration_num_threads_ = root.get<int>("registration_tracker.registration_num_threads");
        config.registration_transform_epsilon_ = root.get<double>("registration_tracker.registration_transform_epsilon");
        config.registration_max_iterations_ = root.get<int>("registration_tracker.registration_max_iterations");
        config.registration_max_corr_distance_ = root.get<double>("registration_tracker.registration_max_corr_distance");
        config.registration_corr_randomness_ = root.get<int>("registration_tracker.registration_corr_randomness");
        config.registration_resolution_ = root.get<double>("registration_tracker.registration_resolution");
        config.registration_use_reciprocal_corr_ = root.get<bool>("registration_tracker.registration_use_reciprocal_corr");
        config.registration_max_optimizer_iterations_ = root.get<int>("registration_tracker.registration_max_optimizer_iterations");
        config.registration_nn_search_method_ = root.get<std::string>("registration_tracker.registration_nn_search_method");
        config.ndt_resolution_ = root.get<float>("registration_tracker.ndt_resolution");
        config.verbose_ = root.get<bool>("registration_tracker.verbose");

        return config;
    }

    LoopDetector::Configuration parse_loop_detector_configuration(const std::string& filename) {
        boost::property_tree::ptree root;
        LoopDetector::Configuration config;
        try {
            boost::property_tree::read_json(filename, root);
        } catch(std::exception &e) {
            return config;
        }

        config.distance_threshold_ = root.get<double>("loop_detector.distance_threshold");
        config.accumulated_distance_threshold_ = root.get<double>("loop_detector.accumulated_distance_threshold");
        config.distance_from_last_edge_threshold_ = root.get<double>("loop_detector.distance_from_last_edge_threshold");
        config.use_scan_context_ = root.get<bool>("loop_detector.use_scan_context");
        config.best_k_contexts_ = root.get<int>("loop_detector.best_k_contexts");
        config.fitness_score_max_range_ = root.get<double>("loop_detector.fitness_score_max_range");
        config.fitness_score_threshold_ = root.get<double>("loop_detector.fitness_score_threshold");
        config.verbose_ = root.get<bool>("loop_detector.verbose");

        return config;
    }

    Registration::Configuration parse_registration_loop_detector_configuration(const std::string& filename) {
        boost::property_tree::ptree root;
        Registration::Configuration config;
        try {
            boost::property_tree::read_json(filename, root);
        } catch(std::exception &e) {
            return config;
        }

        config.registration_method_name_ = root.get<std::string>("registration_loop_detector.registration_method_name");
        config.registration_num_threads_ = root.get<int>("registration_loop_detector.registration_num_threads");
        config.registration_transform_epsilon_ = root.get<double>("registration_loop_detector.registration_transform_epsilon");
        config.registration_max_iterations_ = root.get<int>("registration_loop_detector.registration_max_iterations");
        config.registration_max_corr_distance_ = root.get<double>("registration_loop_detector.registration_max_corr_distance");
        config.registration_corr_randomness_ = root.get<int>("registration_loop_detector.registration_corr_randomness");
        config.registration_resolution_ = root.get<double>("registration_loop_detector.registration_resolution");
        config.registration_use_reciprocal_corr_ = root.get<bool>("registration_loop_detector.registration_use_reciprocal_corr");
        config.registration_max_optimizer_iterations_ = root.get<int>("registration_loop_detector.registration_max_optimizer_iterations");
        config.registration_nn_search_method_ = root.get<std::string>("registration_loop_detector.registration_nn_search_method");
        config.ndt_resolution_ = root.get<float>("registration_loop_detector.ndt_resolution");
        config.verbose_ = root.get<bool>("registration_loop_detector.verbose");

        return config;
    }

    InformationMatrixCalculator::Configuration parse_information_matrix_calculator(const std::string& filename) {
        boost::property_tree::ptree root;
        InformationMatrixCalculator::Configuration config;
        try {
            boost::property_tree::read_json(filename, root);
        } catch(std::exception &e) {
            return config;
        }

        config.constant_information_matrix_ = root.get<bool>("information_matrix_calculator.constant_information_matrix");
        config.translation_constant_stddev_ = root.get<double>("information_matrix_calculator.translation_constant_stddev");
        config.rotation_constant_stddev_ = root.get<double>("information_matrix_calculator.rotation_constant_stddev");
        config.variance_gain_ = root.get<double>("information_matrix_calculator.variance_gain");
        config.minimum_translation_stddev_ = root.get<double>("information_matrix_calculator.minimum_translation_stddev");
        config.maximum_translation_stddev_ = root.get<double>("information_matrix_calculator.maximum_translation_stddev");
        config.minimum_rotation_stddev_ = root.get<double>("information_matrix_calculator.minimum_rotation_stddev");
        config.maximum_rotation_stddev_ = root.get<double>("information_matrix_calculator.maximum_rotation_stddev");
        config.fitness_score_threshold_ = root.get<double>("information_matrix_calculator.fitness_score_threshold");
        config.verbose_ = root.get<bool>("information_matrix_calculator.verbose");
    }

    BackendHandler::Configuration parse_backend_handler_configuration(const std::string& filename) {
        boost::property_tree::ptree root;
        BackendHandler::Configuration config;
        try {
            boost::property_tree::read_json(filename, root);
        } catch(std::exception &e) {
            return config;
        }

        config.use_anchor_graph_node_ = root.get<bool>("backend_handler.use_anchor_graph_node");
        int i = 0;
        for(boost::property_tree::ptree::value_type &val : root.get_child("backend_handler.anchor_graph_node_stddev")) {
            config.anchor_graph_node_stddev_[i] = val.second.get_value<double>();
            i++;
        }
        config.fix_first_graph_node_ = root.get<bool>("backend_handler.fix_first_graph_node");
        config.first_graph_node_adaptive_ = root.get<bool>("backend_handler.first_graph_node_adaptive");
        config.odometry_edge_robust_kernel_ = root.get<std::string>("backend_handler.odometry_edge_robust_kernel");
        config.odometry_edge_robust_kernel_size_ = root.get<double>("backend_handler.odometry_edge_robust_kernel_size");
        config.floor_edge_stddev_ = root.get<double>("backend_handler.floor_edge_stddev");
        config.floor_edge_robust_kernel_ = root.get<std::string>("backend_handler.floor_edge_robust_kernel");
        config.floor_edge_robust_kernel_size_ = root.get<double>("backend_handler.floor_edge_robust_kernel_size");
        config.imu_acceleration_enabled_ = root.get<bool>("backend_handler.imu_acceleration_enabled");
        config.imu_orientation_enabled_ = root.get<bool>("backend_handler.imu_orientation_enabled");
        i = 0;
        for(boost::property_tree::ptree::value_type &val : root.get_child("backend_handler.imu_to_lidar_rotation")) {
            config.imu_to_lidar_rotation_[i] = val.second.get_value<double>();
            i++;
        }
        config.imu_acceleration_stddev_ = root.get<double>("backend_handler.imu_acceleration_stddev");
        config.imu_orientation_stddev_ = root.get<double>("backend_handler.imu_orientation_stddev");
        config.imu_acceleration_edge_robust_kernel_ = root.get<std::string>("backend_handler.imu_acceleration_edge_robust_kernel");
        config.imu_acceleration_edge_robust_kernel_size_ = root.get<double>("backend_handler.imu_acceleration_edge_robust_kernel_size");
        config.imu_orientation_edge_robust_kernel_ = root.get<std::string>("backend_handler.imu_orientation_edge_robust_kernel");
        config.imu_orientation_edge_robust_kernel_size_ = root.get<double>("backend_handler.imu_orientation_edge_robust_kernel_size");
        config.gnss_enabled_ = root.get<bool>("backend_handler.gnss_enabled");
        i = 0;
        for(boost::property_tree::ptree::value_type &val : root.get_child("backend_handler.gnss_to_lidar_translation")) {
            config.gnss_to_lidar_translation_[i] = val.second.get_value<double>();
            i++;
        }
        config.gnss_edge_xy_stddev_ = root.get<double>("backend_handler.gnss_edge_xy_stddev");
        config.gnss_edge_z_stddev_ = root.get<double>("backend_handler.gnss_edge_z_stddev");
        config.gnss_edge_robust_kernel_ = root.get<std::string>("backend_handler.gnss_edge_robust_kernel");
        config.gnss_edge_robust_kernel_size_ = root.get<double>("backend_handler.gnss_edge_robust_kernel_size");
        config.loop_edge_robust_kernel_ = root.get<std::string>("backend_handler.loop_edge_robust_kernel");
        config.loop_edge_robust_kernel_size_ = root.get<double>("backend_handler.loop_edge_robust_kernel_size");
        config.optimizer_iterations_ = root.get<unsigned short>("backend_handler.optimizer_iterations");
        config.maximum_unoptimized_keyframes_ = root.get<unsigned short>("backend_handler.maximum_unoptimized_keyframes");
        config.max_keyframes_per_update_ = root.get<unsigned short>("backend_handler.max_keyframes_per_update");
        config.send_to_observers_ = root.get<bool>("backend_handler.send_to_observers");
        config.verbose_ = root.get<bool>("backend_handler.verbose");

        return config;
    }
}
