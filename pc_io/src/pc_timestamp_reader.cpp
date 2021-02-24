/** @file pc_timestamp_reader.cpp
 * @brief Definition of class PCTimestampReader
 */

#include <pc_timestamp_reader.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>

/* Reads the timestamps from file, expressed in NANOSECONDS. */
void PCTimestampReader::read_timestamps(const std::string& filename, std::vector<uint64_t>& timestamp_vec) {
    std::ifstream timestamps_file(filename.c_str(), std::ios::in);
    std::string line;

    while(std::getline(timestamps_file, line)) {
        std::stringstream line_stream(line);

        std::string timestamp_string = line_stream.str();
        std::tm t = {};
        t.tm_year = std::stoi(timestamp_string.substr(0,4)) - 1900;
        t.tm_mon = std::stoi(timestamp_string.substr(5,2));
        t.tm_mday = std::stoi(timestamp_string.substr(8,2));
        t.tm_hour = std::stoi(timestamp_string.substr(11,2));
        t.tm_min = std::stoi(timestamp_string.substr(14,2));
        t.tm_sec = std::stoi(timestamp_string.substr(17,2));

        static const uint64_t k_seconds_to_nanoseconds = 1e9;
        time_t time_since_epoch = timegm(&t);

        uint64_t timestamp = time_since_epoch * k_seconds_to_nanoseconds + std::stoi(timestamp_string.substr(20,9));

        timestamp_vec.push_back(timestamp);
    }

    std::ostringstream to_print;
    to_print << "[TimestampReader][read_timestamps] timestamps: " << timestamp_vec.front() << " - " << timestamp_vec.back() << "\n";
    std::cout << to_print.str();
}
