/** @file pc_timestamp_reader.h
* @brief Declaration of class PCTimestampReader
*/

#ifndef ARTSLAM_PC_TIMESTAMP_READER_H
#define ARTSLAM_PC_TIMESTAMP_READER_H


#include <string>
#include <vector>

/**
 * @class PCTimestampReader
 * @brief This class is used to read the timestamps associated to multiple point clouds.
 * @author Matteo Frosi
 */
class PCTimestampReader {
public:
    /**
     * @brief Class constructor
     */
    PCTimestampReader() {};

    /**
     * @brief Reads the timestamps from file, expressed in NANOSECONDS.
     * @param filename Name of the file containing the timestamps.
     * @param timestamp_vec Vector of timestamps, to be filled.
     */
    void read_timestamps(const std::string& filename, std::vector<uint64_t>& timestamp_vec);
};


#endif //ARTSLAM_PC_TIMESTAMP_READER_H
