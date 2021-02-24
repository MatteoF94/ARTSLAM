#include <iostream>
#include <bin_io.h>
#include <pc_timestamp_reader.h>

#include <slam_types.h>

#include <configs_reader.h>
#include <prefilterer_config.h>
#include <tracker_config.h>
#include <registration_config.h>
#include <floor_detector_config.h>
#include <loop_detector_config.h>
#include <info_matrix_calculator_config.h>

#include <prefilterer.h>
#include <tracker.h>
#include <registration.h>
#include <floor_detector.h>
#include <backend.h>
#include <loop_detector.h>
#include <info_matrix_calculator.h>

#include <ukf.h>
#include <kitti_reader.h>

#include <g2o/edge_se3_plane.hpp>
#include <graph_handler.h>

// method used to get all the filenames
std::vector<std::string> get_filepaths(boost::filesystem::path const & root, std::string const & ext) {
    std::vector<std::string> paths;

    if(boost::filesystem::exists(root) && boost::filesystem::is_directory(root)) {
        for(auto const & entry : boost::filesystem::recursive_directory_iterator(root)) {
            if(boost::filesystem::is_regular_file(entry) && entry.path().extension() == ext) {
                paths.emplace_back(entry.path().string());
            }
        }
    }

    std::sort(paths.begin(), paths.end());
    return paths;
}

int main() {

    ConfigsReader configs_reader;
    /*UKFConfig ukf_config = configs_reader.load_ukf_config("../configs/ukf_config.txt");

    std::vector<std::string> imugnss_filepath = get_filepaths("/home/matteo/Downloads/2011_09_26/2011_09_26_drive_0005_sync/oxts/data", ".txt");
    KITTIReader kitti_reader;
    std::vector<uint64_t> imugnss_stamps;
    kitti_reader.read_timestamps("/home/matteo/Downloads/2011_09_26/2011_09_26_drive_0005_sync/oxts/timestamps.txt", imugnss_stamps);
    RobotMeasurement2D rm2d;
    IMUControlInput imu_ci;

    UKF2D ukf2D(&ukf_config);
    kitti_reader.read_imu_gnss(imugnss_filepath[0], imu_ci, rm2d);
    imu_ci.timestamp = imugnss_stamps[0];
    rm2d.timestamp = imugnss_stamps[0];
    ukf2D.initialize_state(0.0, 0.0, rm2d.measurement_vector(2), rm2d.measurement_vector(3), rm2d.measurement_vector(4));
    ukf2D.perform_batch_step(imu_ci, rm2d);

    for(int i = 0; i < imugnss_stamps.size() - 1; i++) {
        kitti_reader.read_imu_gnss(imugnss_filepath[i], imu_ci, rm2d);
        RobotMeasurement2D tmp_meas;
        IMUControlInput tmp_control;
        kitti_reader.read_imu_gnss(imugnss_filepath[i+1], tmp_control, tmp_meas);
        imu_ci.timestamp = imugnss_stamps[i];
        tmp_meas.timestamp = imugnss_stamps[i+1];
        ukf2D.perform_batch_step(imu_ci,tmp_meas);
    }

    std::string pos_file = "/home/matteo/Desktop/poses.txt";
    ukf2D.save_trajectory(pos_file);

    return 0;*/

    // get the sorted filepaths to load the point clouds
    std::vector<std::string> pc_filepaths = get_filepaths("/home/matteo/Downloads/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data", ".bin");
    //std::vector<std::string> pc_filepaths = get_filepaths("/mnt/8b3dfc8f-1ab3-4fa8-93ca-45ba9f9f8564/matteo/kitti/2011_10_03/2011_10_03_drive_0027_sync/velodyne_points/data", ".bin");
    //std::vector<std::string> pc_filepaths = get_filepaths("/mnt/8b3dfc8f-1ab3-4fa8-93ca-45ba9f9f8564/matteo/kitti/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/data", ".bin");


    // load the config files
    PrefiltererConfig prefilterer_config = configs_reader.load_prefilterer_config("../configs/prefilterer_config.txt");
    TrackerConfig tracker_config = ConfigsReader::load_tracker_config("../configs/tracker_config.txt");
    RegistrationConfig registration_config = ConfigsReader::load_registration_config("../configs/registration_config.txt");
    FloorDetectorConfig floor_detector_config = ConfigsReader::load_floor_detector_config("../configs/floor_detector_config.txt");

    // reading the timestamps
    PCTimestampReader pc_timestamp_reader;
    std::vector<uint64_t> pc_timestamps;
    pc_timestamp_reader.read_timestamps("/home/matteo/Desktop/timestamps.txt", pc_timestamps);
    //pc_timestamp_reader.read_timestamps("/mnt/8b3dfc8f-1ab3-4fa8-93ca-45ba9f9f8564/matteo/kitti/2011_10_03/2011_10_03_drive_0027_sync/velodyne_points/timestamps.txt", pc_timestamps);
    //pc_timestamp_reader.read_timestamps("/mnt/8b3dfc8f-1ab3-4fa8-93ca-45ba9f9f8564/matteo/kitti/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/timestamps.txt", pc_timestamps);

    // create the SLAM objects
    Prefilterer prefilterer(&prefilterer_config);

    Registration registration_tracker(&registration_config);
    Tracker tracker(&tracker_config);
    tracker.set_registration(&registration_tracker);

    FloorDetector floor_detector(&floor_detector_config);

    Registration registration_loop_detector(&registration_config);
    LoopDetector loop_detector; // can have a config
    loop_detector.set_registration(&registration_loop_detector);
    InfoMatrixCalculator info_matrix_calculator;
    GraphHandler graph_handler;
    Backend backend;
    backend.set_graph_handler(&graph_handler);
    backend.set_info_matrix_calculator(&info_matrix_calculator);
    backend.set_loop_detector(&loop_detector);

    // create ukf objects
    UKFConfig ukf_config = ConfigsReader::load_ukf_config("../configs/ukf_config.txt");

    std::vector<std::string> imugnss_filepath = get_filepaths("/home/matteo/Downloads/2011_09_26/2011_09_26_drive_0005_sync/oxts/data", ".txt");
    //std::vector<std::string> imugnss_filepath = get_filepaths("/mnt/8b3dfc8f-1ab3-4fa8-93ca-45ba9f9f8564/matteo/kitti/2011_09_30/2011_09_30_drive_0027_sync/oxts/data", ".txt");
    //std::vector<std::string> imugnss_filepath = get_filepaths("/mnt/8b3dfc8f-1ab3-4fa8-93ca-45ba9f9f8564/matteo/kitti/2011_10_03/2011_10_03_drive_0027_sync/oxts/data", ".txt");
    KITTIReader kitti_reader;
    std::vector<uint64_t> imugnss_stamps;
    pc_timestamp_reader.read_timestamps("/home/matteo/Downloads/2011_09_26/2011_09_26_drive_0005_sync/oxts/timestamps.txt", imugnss_stamps);
    //pc_timestamp_reader.read_timestamps("/mnt/8b3dfc8f-1ab3-4fa8-93ca-45ba9f9f8564/matteo/kitti/2011_09_30/2011_09_30_drive_0027_sync/oxts/timestamps.txt", imugnss_stamps);
    //pc_timestamp_reader.read_timestamps("/mnt/8b3dfc8f-1ab3-4fa8-93ca-45ba9f9f8564/matteo/kitti/2011_10_03/2011_10_03_drive_0027_sync/oxts/timestamps.txt", imugnss_stamps);
    RobotMeasurement2D rm2d;
    IMUControlInput imu_ci;

    UKF2D ukf2D(&ukf_config);
    ukf2D.register_prior_odom_observer(&tracker);
    kitti_reader.read_imu_gnss(imugnss_filepath[0], imu_ci, rm2d);
    imu_ci.timestamp = imugnss_stamps[0];
    rm2d.timestamp = imugnss_stamps[0];
    ukf2D.initialize_state(0.0, 0.0, rm2d.measurement_vector(2), rm2d.measurement_vector(3), rm2d.measurement_vector(4));
    ukf2D.perform_batch_step(imu_ci, rm2d);

    for(int i = 0; i < imugnss_stamps.size() - 1; i++) {
        kitti_reader.read_imu_gnss(imugnss_filepath[i], imu_ci, rm2d);
        RobotMeasurement2D tmp_meas;
        IMUControlInput tmp_control;
        kitti_reader.read_imu_gnss(imugnss_filepath[i+1], tmp_control, tmp_meas);
        imu_ci.timestamp = imugnss_stamps[i];
        tmp_meas.timestamp = imugnss_stamps[i+1];
        ukf2D.perform_batch_step(imu_ci,tmp_meas);
    }

    // register the different observers
    prefilterer.register_filtered_cloud_observer(&floor_detector);
    prefilterer.register_filtered_cloud_observer(&tracker);

    tracker.register_keyframe_observer(&backend);
    floor_detector.register_floor_coeffs_observer(&backend);

    // point clouds reader objects
    BinIO binIO; // reads .bin point clouds

    // offline SLAM body
    for(int i = 0; i < pc_timestamps.size(); ++i) {
        pcl::PointCloud<PointT>::Ptr curr_pc_ptr(new pcl::PointCloud<PointT>());

        curr_pc_ptr->header.frame_id = "";
        curr_pc_ptr->header.seq = i;
        curr_pc_ptr->header.stamp = pc_timestamps[i];
        binIO.read_point_cloud(pc_filepaths[i], curr_pc_ptr);

        pcl::PointCloud<PointT>::ConstPtr curr_pc_constptr(curr_pc_ptr);

        prefilterer.update(curr_pc_constptr);
        usleep(50000);
    }
    sleep(10);
    backend.optimize();
    std::cout << "main finished ..." << std::endl;
    backend.save_data("/home/matteo/Desktop/ukf_prova");
    return 0;
}
