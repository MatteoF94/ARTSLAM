/** @file configs_reader.h
 * @brief Declaration of class ConfigsReader
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_CONFIGS_READER_H
#define ARTSLAM_CONFIGS_READER_H


#include <prefilterer_config.h>
#include <tracker_config.h>
#include <registration_config.h>
#include <floor_detector_config.h>
#include <info_matrix_calculator_config.h>
#include <loop_detector_config.h>

#include <ukf_config.h>

/**
 * @class ConfigsReader
 * @brief This class is used to read the various config files associated to the system.
 */
class ConfigsReader {
public:
    /**
     * @brief Class constructor.
     */
    ConfigsReader();

    /**
     * @brief Loads the configuration file associated to the prefilterer configuration object.
     * @param config_path The prefilterer configuration file path.
     * @return The PrefiltererConfig object created from the corresponding configuration file.
     */
    PrefiltererConfig load_prefilterer_config(const std::string& config_path);

    /**
     * @brief Loads the configuration file associated to the tracker configuration object.
     * @param config_path The tracker configuration file path.
     * @return The TrackerConfig object created from the corresponding configuration file.
     */
    static TrackerConfig load_tracker_config(const std::string& config_path);

    /**
     * @brief Loads the configuration file associated to the registration configuration object.
     * @param config_path The registration configuration file path.
     * @return The RegistrationConfig object created from the corresponding configuration file.
     */
    static RegistrationConfig load_registration_config(const std::string& config_path);

    /**
     * @brief Loads the configuration file associated to the floor detector configuration object.
     * @param config_path The floor detector configuration file path.
     * @return The FloorDetectorConfig object created from the corresponding configuration file.
     */
    static FloorDetectorConfig load_floor_detector_config(const std::string& config_path);

    /**
     * @brief Loads the configuration file associated to the information matrix calculator configuration object.
     * @param config_path The information matrix calculator configuration file path.
     * @return The InfoMatrixCalculatorConfig object created from the corresponding configuration file.
     */
    static InfoMatrixCalculatorConfig load_info_matrix_calculator_config(const std::string& config_path);

    /**
     * @brief Loads the configuration file associated to the loop detector configuration object.
     * @param config_path The loop detector condiguration file path.
     * @return The LoopDetectorConfig object created from the corresponding configuration file.
     */
    static LoopDetectorConfig load_loop_detector_config(const std::string& config_path);

    /**
     * @brief Loads the configuration file associated to the UKF configuration object.
     * @param config_path The ukf configuration file path.
     * @return The UKFConfig object created from the corresponding configuration file.
     */
    static UKFConfig load_ukf_config(const std::string& config_path);
};


#endif //ARTSLAM_CONFIGS_READER_H
