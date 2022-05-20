#include <iostream>
#include <filesystem>
#include <algorithm>
#include <artslam_io/pointcloud_io.h>
#include <artslam_io/kitti_reader.hpp>
#include "configuration_parser.h"
#include "prefilterer.h"
#include "registration.h"
#include "tracker.h"
#include "ground_detector.h"
#include "graph_handler.h"
#include "information_matrix_calculator.h"
#include "loop_detector.h"
#include "backend_handler.h"

using namespace artslam::laser3d;
using namespace artslam::core::io;

void get_filepaths(const std::string& path, const std::string& extension, std::vector<std::string>& filepaths) {
    for(const auto& p : std::filesystem::directory_iterator(path)) {
        if(p.is_regular_file()) {
            if(p.path().extension().string() == extension) {
                filepaths.emplace_back(p.path());
            }
        }
    }

    std::sort(filepaths.begin(), filepaths.end());
}

int main(int argc, char** argv) {
    std::string config_file = "/home/matteo/catkin_ws/src/artslam_laser_3d/configs/KITTI_long.json";
    std::string save_path = "/home/matteo/results/";
    std::vector<std::string> slam_paths = parse_slam_paths(config_file);

    // get the filepaths corresponding to the pointclouds
    std::vector<std::string> pointclouds_filepaths;
    get_filepaths(slam_paths[0], ".bin", pointclouds_filepaths);

    // get the filepaths corresponding to IMU and GPS data (KITTI)
    std::vector<std::string> oxts_filepaths;
    get_filepaths(slam_paths[2], ".txt", oxts_filepaths);

    // read the timestamps for both pointclouds, IMU and GPS data (KITTI)
    KITTI_Reader kitti_reader({false, boost::log::trivial::trace});
    std::vector<uint64_t> timestamps, oxts_timestamps;
    kitti_reader.read_timestamps(slam_paths[1], timestamps);
    kitti_reader.read_timestamps(slam_paths[3], oxts_timestamps);

    // prefilterer
    Prefilterer::Configuration prefilterer_configuration = parse_prefilterer_configuration(config_file);
    Prefilterer prefilterer(prefilterer_configuration);

    // tracker
    Registration::Configuration registration_tracker_configuration = parse_registration_tracker_configuration(config_file);
    Registration registration(registration_tracker_configuration);
    Tracker::Configuration tracker_configuration = parse_tracker_configuration(config_file);
    Tracker tracker(tracker_configuration, registration.registration_method());

    // ground detector
    GroundDetector::Configuration ground_detector_configuration = parse_ground_detector_configuration(config_file);
    GroundDetector ground_detector(ground_detector_configuration);

    // graph handler
    GraphHandler graph_handler;

    // information matrix calculator
    InformationMatrixCalculator::Configuration information_matrix_calculator_configuration = {false, 0.5, 0.1, 20.0, 0.1, 5.0, 0.05, 0.2, 2.5, true};
    InformationMatrixCalculator information_matrix_calculator(information_matrix_calculator_configuration);

    // loop detector
    Registration::Configuration loop_detector_registration_configuration = parse_registration_loop_detector_configuration(config_file);
    Registration loop_detector_registration(loop_detector_registration_configuration);
    LoopDetector::Configuration loop_detector_configuration = parse_loop_detector_configuration(config_file);
    LoopDetector loop_detector(loop_detector_configuration, loop_detector_registration.registration_method());

    // backend handler
    BackendHandler::Configuration backend_handler_configuration = parse_backend_handler_configuration(config_file);
    BackendHandler backend_handler(backend_handler_configuration);
    backend_handler.set_graph_handler(&graph_handler);
    backend_handler.set_information_matrix_calculator(&information_matrix_calculator);
    backend_handler.set_loop_detector(&loop_detector);

    // SLAM pipeline construction
    prefilterer.register_filtered_pointcloud_observer(&tracker);
    prefilterer.register_filtered_pointcloud_observer(&ground_detector);
    tracker.register_keyframe_observer(&backend_handler);
    ground_detector.register_floor_coefficients_observer(&backend_handler);

    // load immediately all IMU and GNSS data
    for(int i = 0; i < oxts_filepaths.size(); i++) {
        IMU3D_MSG::Ptr imu3d_msg = kitti_reader.read_imu(oxts_filepaths[i]);
        imu3d_msg->header_.frame_id_ = "";
        imu3d_msg->header_.sequence_ = i;
        imu3d_msg->header_.timestamp_ = oxts_timestamps[i];
        //prefilterer.update_raw_imu_observer(imu3d_msg);
        //backend_handler.update_raw_imu_observer(imu3d_msg);

        GeoPointStamped_MSG::Ptr gnss_msg = kitti_reader.read_gps(oxts_filepaths[i]);
        gnss_msg->header_.frame_id_ = "";
        gnss_msg->header_.sequence_ = i;
        gnss_msg->header_.timestamp_ = oxts_timestamps[i];
        //backend_handler.update_raw_gnss_observer(gnss_msg);
    }

    // perform SLAM, playing back
    for(int i = 0; i < 200; i++) {
        pcl::PointCloud<Point3I>::Ptr pointcloud = kitti_reader.read_pointcloud(pointclouds_filepaths[i]);
        pointcloud->header.frame_id = "";
        pointcloud->header.seq = i;
        pointcloud->header.stamp = timestamps[i];

        prefilterer.update_raw_pointcloud_observer(pointcloud);
        usleep(100000); // just to simulate data acquisition rate
    }

    sleep(5);

    backend_handler.save_results(save_path);

    std::cout << "[END] Size: " << pointclouds_filepaths.size() << std::endl;
    std::cout << "[END] Prefilterer: " << prefilterer.total_time_ << " - " << prefilterer.count_ << std::endl;
    std::cout << "[END] Tracker: " << tracker.total_time_ << " - " << tracker.count_ << std::endl;
    std::cout << "[END] GroundDetector: " << ground_detector.total_time_ << " - " << ground_detector.count_ << std::endl;
    std::cout << "[END] LoopDetector: " << loop_detector.total_time_ << " - " << loop_detector.count_ << std::endl;
    std::cout << "[END] GraphHandler: " << graph_handler.total_time_ << " - " << graph_handler.count_ << std::endl;
    std::cout << "[END] BackendHandler: " << backend_handler.total_time_ << " - " << backend_handler.count_ << std::endl;

    return 0;
}