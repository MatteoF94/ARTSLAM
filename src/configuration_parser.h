#ifndef ARTSLAM_LASER_3D_CONFIGURATION_PARSER_H
#define ARTSLAM_LASER_3D_CONFIGURATION_PARSER_H

#include <string>
#include <vector>

#include "prefilterer.h"
#include "tracker.h"
#include "ground_detector.h"
#include "registration.h"
#include "loop_detector.h"
#include "information_matrix_calculator.h"
#include "backend_handler.h"

namespace artslam::laser3d {
    std::vector<std::string> parse_slam_paths(const std::string &filename);

    Prefilterer::Configuration parse_prefilterer_configuration(const std::string& filename);

    Tracker::Configuration parse_tracker_configuration(const std::string& filename);

    GroundDetector::Configuration parse_ground_detector_configuration(const std::string& filename);

    Registration::Configuration parse_registration_tracker_configuration(const std::string& filename);

    LoopDetector::Configuration parse_loop_detector_configuration(const std::string& filename);

    Registration::Configuration parse_registration_loop_detector_configuration(const std::string& filename);

    InformationMatrixCalculator::Configuration parse_information_matrix_calculator(const std::string& filename);

    BackendHandler::Configuration parse_backend_handler_configuration(const std::string& filename);
}


#endif //ARTSLAM_LASER_3D_CONFIGURATION_PARSER_H
