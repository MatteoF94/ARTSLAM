/** @file configs_reader.cpp
 * @brief Definition of class ConfigsReader
 */

#include <configs_reader.h>

#include <fstream>
#include <sstream>
#include <iostream>

ConfigsReader::ConfigsReader() = default;

/**
 * This method loads the configuration file associated to the prefilterer configuration object. It reads the file
 * line by line searching for string tokens equal to the PrefiltererConfig object's parameters, filling them.
 */
PrefiltererConfig ConfigsReader::load_prefilterer_config(const std::string& config_path) {
    std::string line;
    std::ifstream config_file(config_path);
    PrefiltererConfig prefilterer_config;

    if(!config_file.is_open()) {
        std::cerr << "could not load prefilter config file, using default config" << std::endl;
        return prefilterer_config;
    }

    while(std::getline(config_file, line)) {
        std::istringstream tokenizer(line);
        std::string data[3];

        std::getline(tokenizer,data[0], ' ');
        std::getline(tokenizer, data[1], ' ');
        std::getline(tokenizer, data[2]);

        if(data[0] == "downsample_method") {
            prefilterer_config.downsample_method = data[2];
            continue;
        }
        if(data[0] == "downsample_resolution") {
            prefilterer_config.downsample_resolution = std::stod(data[2]);
            continue;
        }
        if(data[0] == "sample_size") {
            prefilterer_config.sample_size = std::stoi(data[2]);
        }
        if(data[0] == "outlier_removal_method") {
            prefilterer_config.outlier_removal_method = data[2];
            continue;
        }
        if(data[0] == "statistical_mean_k") {
            prefilterer_config.statistical_mean_k = std::stoi(data[2]);
            continue;
        }
        if(data[0] == "statistical_stddev") {
            prefilterer_config.statistical_stddev = std::stod(data[2]);
            continue;
        }
        if(data[0] == "radius_radius") {
            prefilterer_config.radius_radius = std::stod(data[2]);
            continue;
        }
        if(data[0] == "radius_min_neighbors") {
            prefilterer_config.radius_min_neighbors = std::stoi(data[2]);
            continue;
        }
        if(data[0] == "use_distance_filter") {
            if(data[2] == "true" || data[2] == "TRUE")
                prefilterer_config.use_distance_filter = true;
            if(data[2] == "false" || data[2] == "FALSE")
                prefilterer_config.use_distance_filter = false;
            continue;
        }
        if(data[0] == "distance_near_thresh") {
            prefilterer_config.distance_near_thresh = std::stod(data[2]);
            continue;
        }
        if(data[0] == "distance_far_thresh") {
            prefilterer_config.distance_far_thresh = std::stod(data[2]);
            continue;
        }
    }

    return prefilterer_config;
}

/**
 * This method loads the configuration file associated to the tracker configuration object. It reads the file
 * line by line searching for string tokens equal to the TrackerConfig object's parameters, filling them.
 */
TrackerConfig ConfigsReader::load_tracker_config(const std::string &config_path) {
    std::string line;
    std::ifstream config_file(config_path);
    TrackerConfig tracker_config;

    if(!config_file.is_open()) {
        std::cerr << "could not load tracker config file, using default config" << std::endl;
        return tracker_config;
    }

    while(std::getline(config_file, line)) {
        std::istringstream tokenizer(line);
        std::string data[3];

        std::getline(tokenizer, data[0], ' ');
        std::getline(tokenizer, data[1], ' ');
        std::getline(tokenizer, data[2]);

        if(data[0] == "downsample_method") {
            tracker_config.downsample_method = data[2];
            continue;
        }
        if(data[0] == "downsample_resolution") {
            tracker_config.downsample_resolution = std::stof(data[2]);
            continue;
        }
        if(data[0] == "keyframe_delta_trans") {
            tracker_config.keyframe_delta_trans = std::stod(data[2]);
            continue;
        }
        if(data[0] == "keyframe_delta_angle") {
            tracker_config.keyframe_delta_angle = std::stod(data[2]);
            continue;
        }
        if(data[0] == "keyframe_delta_time") {
            tracker_config.keyframe_delta_time = std::stod(data[2]);
            continue;
        }
        if(data[0] == "transform_thresholding") {
            if(data[2] == "true" || data[2] == "TRUE")
                tracker_config.transform_thresholding = true;
            if(data[2] == "false" || data[2] == "FALSE")
                tracker_config.transform_thresholding = false;
            continue;
        }
        if(data[0] == "max_acceptable_trans") {
            tracker_config.max_acceptable_trans = std::stod(data[2]);
            continue;
        }
        if(data[0] == "max_acceptable_angle") {
            tracker_config.max_acceptable_angle = std::stod(data[2]);
            continue;
        }
        if(data[0] == "num_frames_to_skip") {
            tracker_config.num_frames_to_skip = std::stoi(data[2]);
            continue;
        }
        if(data[0] == "prior_odom_delta_trans") {
            tracker_config.prior_odom_delta_trans = std::stod(data[2]);
            continue;
        }
        if(data[0] == "prior_odom_delta_angle") {
            tracker_config.prior_odom_delta_angle = std::stod(data[2]);
            continue;
        }
    }

    return tracker_config;
}

/**
 * This method loads the configuration file associated to the registration configuration object. It reads the file
 * line by line searching for string tokens equal to the RegistrationConfig object's parameters, filling them.
 */
RegistrationConfig ConfigsReader::load_registration_config(const std::string &config_path) {
    std::string line;
    std::ifstream config_file(config_path);
    RegistrationConfig registration_config;

    if(!config_file.is_open()) {
        std::cerr << "could not load registration config file, using default config" << std::endl;
        return registration_config;
    }

    while(std::getline(config_file, line)) {
        std::istringstream tokenizer(line);
        std::string data[3];

        std::getline(tokenizer, data[0], ' ');
        std::getline(tokenizer, data[1], ' ');
        std::getline(tokenizer, data[2]);

        if(data[0] == "registration_method") {
            registration_config.registration_method = data[2];
            continue;
        }
        if(data[0] == "reg_num_threads") {
            registration_config.reg_num_threads = std::stoi(data[2]);
            continue;
        }
        if(data[0] == "reg_transformation_epsilon") {
            registration_config.reg_transformation_epsilon = std::stod(data[2]);
            continue;
        }
        if(data[0] == "reg_maximum_iterations") {
            registration_config.reg_maximum_iterations = std::stoi(data[2]);
            continue;
        }
        if(data[0] == "reg_max_correspondence_distance") {
            registration_config.reg_max_correspondence_distance = std::stod(data[2]);
            continue;
        }
        if(data[0] == "reg_correspondence_randomness") {
            registration_config.reg_correspondence_randomness = std::stoi(data[2]);
            continue;
        }
        if(data[0] == "reg_resolution") {
            registration_config.reg_resolution = std::stod(data[2]);
            continue;
        }
        if(data[0] == "reg_use_reciprocal_correspondences") {
            if(data[2] == "true" || data[2] == "TRUE")
                registration_config.reg_use_reciprocal_correspondences = true;
            if(data[2] == "false" || data[2] == "FALSE")
                registration_config.reg_use_reciprocal_correspondences = false;
            continue;
        }
        if(data[0] == "reg_max_optimizer_iterations") {
            registration_config.reg_max_optimizer_iterations = std::stoi(data[2]);
            continue;
        }
        if(data[0] == "reg_nn_search_method") {
            registration_config.reg_nn_search_method = data[2];
            continue;
        }
        if(data[0] == "ndt_resolution") {
            registration_config.ndt_resolution = std::stof(data[2]);
            continue;
        }
    }

    return registration_config;
}

/**
 * This method loads the configuration file associated to the floor detector configuration object. It reads the file
 * line by line searching for string tokens equal to the FloorDetectorConfig object's parameters, filling them.
 */
FloorDetectorConfig ConfigsReader::load_floor_detector_config(const std::string &config_path) {
    std::string line;
    std::ifstream config_file(config_path);
    FloorDetectorConfig floor_detector_config;

    if(!config_file.is_open()) {
        std::cerr << "could not load floor detector config file, using default config" << std::endl;
        return floor_detector_config;
    }

    while(std::getline(config_file, line)) {
        std::istringstream tokenizer(line);
        std::string data[3];

        std::getline(tokenizer, data[0], ' ');
        std::getline(tokenizer, data[1], ' ');
        std::getline(tokenizer, data[2]);

        if(data[0] == "tilt_deg") {
            floor_detector_config.tilt_deg = std::stod(data[2]);
            continue;
        }
        if(data[0] == "sensor_height") {
            floor_detector_config.sensor_height = std::stod(data[2]);
            continue;
        }
        if(data[0] == "height_clip_range") {
            floor_detector_config.height_clip_range = std::stod(data[2]);
            continue;
        }
        if(data[0] == "floor_pts_thresh") {
            floor_detector_config.floor_pts_thresh = std::stoi(data[2]);
            continue;
        }
        if(data[0] == "floor_normal_thresh") {
            floor_detector_config.floor_normal_thresh = std::stod(data[2]);
            continue;
        }
        if(data[0] == "use_normal_filtering") {
            if(data[2] == "true" || data[2] == "TRUE")
                floor_detector_config.use_normal_filtering = true;
            if(data[2] == "false" || data[2] == "FALSE")
                floor_detector_config.use_normal_filtering = false;
            continue;
        }
        if(data[0] == "normal_filter_thresh") {
            floor_detector_config.normal_filter_thresh = std::stod(data[2]);
            continue;
        }
    }

    return floor_detector_config;
}

InfoMatrixCalculatorConfig ConfigsReader::load_info_matrix_calculator_config(const std::string &config_path) {
    std::string line;
    std::ifstream config_file(config_path);
    InfoMatrixCalculatorConfig info_matrix_calculator_config;

    if(!config_file.is_open()) {
        std::cerr << "could not load floor detector config file, using default config" << std::endl;
        return info_matrix_calculator_config;
    }

    while(std::getline(config_file, line)) {
        std::istringstream tokenizer(line);
        std::string data[3];

        std::getline(tokenizer, data[0], ' ');
        std::getline(tokenizer, data[1], ' ');
        std::getline(tokenizer, data[2]);

        if(data[0] == "use_const_info_matrix") {
            if(data[2] == "true" || data[2] == "TRUE")
                info_matrix_calculator_config.use_const_info_matrix = true;
            if(data[2] == "false" || data[2] == "FALSE")
                info_matrix_calculator_config.use_const_info_matrix = false;
            continue;
        }
        if(data[0] == "const_stddev_x") {
            info_matrix_calculator_config.const_stddev_x = std::stod(data[2]);
            continue;
        }
        if(data[0] == "const_stddev_q") {
            info_matrix_calculator_config.const_stddev_q = std::stod(data[2]);
            continue;
        }
        if(data[0] == "var_gain_a") {
            info_matrix_calculator_config.var_gain_a = std::stod(data[2]);
            continue;
        }
        if(data[0] == "min_stddev_x") {
            info_matrix_calculator_config.min_stddev_x = std::stoi(data[2]);
            continue;
        }
        if(data[0] == "max_stddev_x") {
            info_matrix_calculator_config.max_stddev_x = std::stod(data[2]);
            continue;
        }
        if(data[0] == "min_stddev_q") {
            info_matrix_calculator_config.min_stddev_q = std::stod(data[2]);
            continue;
        }
        if(data[0] == "max_stddev_q") {
            info_matrix_calculator_config.max_stddev_q = std::stod(data[2]);
            continue;
        }
        if(data[0] == "fitness_score_thresh") {
            info_matrix_calculator_config.fitness_score_thresh = std::stod(data[2]);
            continue;
        }
    }

    return info_matrix_calculator_config;
}

UKFConfig ConfigsReader::load_ukf_config(const std::string &config_path) {
    std::string line;
    std::ifstream config_file(config_path);
    UKFConfig ukf_config;

    if(!config_file.is_open()) {
        std::cerr << "could not load registration config file, using default config" << std::endl;
        return ukf_config;
    }

    while(std::getline(config_file, line)) {
        std::istringstream tokenizer(line);
        std::string data[3];

        std::getline(tokenizer, data[0], ' ');
        std::getline(tokenizer, data[1], ' ');
        std::getline(tokenizer, data[2]);

        if (data[0] == "noise_var_pos") {
            ukf_config.noise_var_pos = std::stod(data[2]);
            continue;
        }
        if (data[0] == "noise_var_quat") {
            ukf_config.noise_var_quat = std::stod(data[2]);
            continue;
        }
        if (data[0] == "noise_var_vel") {
            ukf_config.noise_var_vel = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_easting") {
            ukf_config.init_easting = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_northing") {
            ukf_config.init_northing = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_elevation") {
            ukf_config.init_elevation = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_quat_w") {
            ukf_config.init_quat_w = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_quat_x") {
            ukf_config.init_quat_x = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_quat_y") {
            ukf_config.init_quat_y = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_quat_z") {
            ukf_config.init_quat_z = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_forward_vel") {
            ukf_config.init_forward_vel = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_leftward_vel") {
            ukf_config.init_leftward_vel = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_upward_vel") {
            ukf_config.init_upward_vel = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_var_pos") {
            ukf_config.init_var_pos = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_var_quat") {
            ukf_config.init_var_quat = std::stod(data[2]);
            continue;
        }
        if (data[0] == "init_var_vel") {
            ukf_config.init_var_vel = std::stod(data[2]);
            continue;
        }
        if (data[0] == "noise_var_pos_meas") {
            ukf_config.noise_var_pos_meas = std::stod(data[2]);
            continue;
        }
        if (data[0] == "noise_var_quat_meas") {
            ukf_config.noise_var_quat_meas = std::stod(data[2]);
            continue;
        }
        if (data[0] == "noise_var_vel_meas") {
            ukf_config.noise_var_vel_meas = std::stod(data[2]);
            continue;
        }
        if (data[0] == "control_var_ang_vel") {
            ukf_config.control_var_ang_vel = std::stod(data[2]);
            continue;
        }
        if (data[0] == "control_var_accel") {
            ukf_config.control_var_accel = std::stod(data[2]);
            continue;
        }
        if (data[0] == "num_iterations") {
            ukf_config.num_iterations = std::stoi(data[2]);
            continue;
        }
        if (data[0] == "alpha") {
            ukf_config.alpha = std::stod(data[2]);
            continue;
        }
        if (data[0] == "beta") {
            ukf_config.beta = std::stod(data[2]);
            continue;
        }
        if (data[0] == "k") {
            ukf_config.k = std::stoi(data[2]);
            continue;
        }
    }

    return ukf_config;
}