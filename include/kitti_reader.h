//
// Created by matteo on 15/02/21.
//

#ifndef ARTSLAM_KITTI_READER_H
#define ARTSLAM_KITTI_READER_H


#include <robot_control_input.h>
#include <robot_measurement.h>

class KITTIReader {
public:
    KITTIReader();

    void read_imu_gnss(const std::string& filename, IMUControlInput& control_input, RobotMeasurement2D& measurement);

    void read_timestamps(const std::string& filename, std::vector<uint64_t>& timestamp_vec);
};


#endif //ARTSLAM_KITTI_READER_H
