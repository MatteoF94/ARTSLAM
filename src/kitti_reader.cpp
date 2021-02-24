//
// Created by matteo on 15/02/21.
//

#include <kitti_reader.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>

KITTIReader::KITTIReader() {
    std::cout << "prova" << std::endl;
}

void KITTIReader::read_imu_gnss(const std::string& filename, IMUControlInput& control_input, RobotMeasurement2D& measurement) {
    std::ifstream timestamps_file(filename.c_str(), std::ios::in);
    std::string line;

    if(std::getline(timestamps_file, line)) {
        std::istringstream ss(line);
        std::vector<double> values(std::istream_iterator<double>{ss}, std::istream_iterator<double>{});
        std::istringstream line_stream(line);

        measurement.measurement_vector(0) = values[0];
        measurement.measurement_vector(1) = values[1];
        double altitude = values[2];

        double roll = values[3];
        double pitch = values[4];
        measurement.measurement_vector(2) = values[5];   // the yaw

        double vn = values[6];
        double ve = values[7];
        measurement.measurement_vector(3) = values[8];
        measurement.measurement_vector(4) = values[9];
        double vu = values[10];

        double ax = values[11];
        double ay = values[12];
        double az = values[13];
        control_input.accel_x = values[14];
        control_input.accel_y = values[15];
        control_input.accel_z = values[16];

        double wx = values[17];
        double wy = values[18];
        double wz = values[19];
        control_input.gyro_x = values[20];
        control_input.gyro_y = values[21];
        control_input.gyro_z = values[22];
    }
}

void KITTIReader::read_timestamps(const std::string& filename, std::vector<uint64_t>& timestamp_vec) {
    std::ifstream timestamps_file(filename.c_str(), std::ios::in);
    std::string line;

    while(std::getline(timestamps_file, line)) {
        std::stringstream line_stream(line);

        std::string timestamp_string = line_stream.str();
        std::tm t = {};
        t.tm_year = std::stoi(timestamp_string.substr(0,4)) - 1900;
        t.tm_mon = std::stoi(timestamp_string.substr(5,2)) - 1;
        t.tm_mday = std::stoi(timestamp_string.substr(8,2));
        t.tm_hour = std::stoi(timestamp_string.substr(11,2));
        t.tm_min = std::stoi(timestamp_string.substr(14,2));
        t.tm_sec = std::stoi(timestamp_string.substr(17,2));
        t.tm_isdst = -1;

        setenv("TZ", "UTC", 1);

        static const uint64_t k_seconds_to_nanoseconds = 1e9;
        uint64_t time_since_epoch = mktime(&t);

        uint64_t timestamp = time_since_epoch * k_seconds_to_nanoseconds + std::stoi(timestamp_string.substr(20,9));

        timestamp_vec.push_back(timestamp);
    }

    std::ostringstream to_print;
    to_print << "[TimestampReader][read_timestamps] timestamps: " << timestamp_vec.front() << " - " << timestamp_vec.back() << "\n";
    std::cout << to_print.str();
}