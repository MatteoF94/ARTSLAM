/** @file slam_types.h
 * @brief Types and structures used in the SLAM system
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_SLAM_TYPES_H
#define ARTSLAM_SLAM_TYPES_H


#include <pcl/point_types.h>
#include <string>
#include <Eigen/Dense>

/**
 * @brief Describes a 3D point having X, Y and Z coordinates and intensity I.
 */
typedef pcl::PointXYZI PointT;

/**
 * @brief Contains general information for a sensor acquisition.
 */
typedef struct {
    uint32_t sequence;      /**< Sequence ID of the measurement */
    uint64_t timestamp;     /**< Time of the measurement in nanoseconds */
    std::string frame_id;   /**< Frame associated to the measurement, useful if working with the ROS framework */
} Header;

/**
 * @brief Contains information about an IMU measurements.
 */
typedef struct ImuMSG_tag{
    Header header;                              /**< General information of the measurement */
    bool has_orientation;                       /**< Whether the orientation is available or not */
    Eigen::Quaterniond orientation;             /**< Orientation measured, in quaternion coordinates */
    double orientation_covariance[9];           /**< Orientation covariance, should be all 0s if unknown */
    bool has_angular_velocity;                  /**< Whether the angular velocity is available or not */
    Eigen::Vector3d angular_velocity;           /**< Angular velocity measured [rad/s] */
    double angular_velocity_covariance[9];      /**< Angular velocity covariance, should be all 0s if unknown */
    bool has_linear_acceleration;               /**< Whether the linear acceleration is available or not */
    Eigen::Vector3d linear_acceleration;        /**< Linear acceleration measured [m/s^2] */
    double linear_acceleration_covariance[9];   /**< Linear acceleration covariance, should be all 0s if unknown */

    using Ptr = std::shared_ptr<struct ImuMSG_tag>;             /**< Pointer to a ImuMSG variable */
    using ConstPtr = std::shared_ptr<const struct ImuMSG_tag>;  /**< Pointer to a constant ImuMSG variable */
} ImuMSG;

/**
 * @brief Contains information about the coefficients of a detected floor.
 */
typedef struct FloorCoeffsMSG_tag {
    Header header;                  /**< General information of the measurement */
    Eigen::Vector4d floor_coeffs;   /**< Estimated floor coefficients */

    using Ptr = std::shared_ptr<struct FloorCoeffsMSG_tag>;             /**< Pointer to a FloorCoeffsMSG variable */
    using ConstPtr = std::shared_ptr<const struct FloorCoeffsMSG_tag>;  /**< Pointer to a constant FloorCoeffsMSG variable */
} FloorCoeffsMSG;

namespace NMEA {
/**
 * @brief Contains information about a GPGGA NMEA message (Global Positioning System Fix Data), where time, position and fix
 * are related data of the receiver.
 */
    typedef struct GpggaMSG_tag {
        Header header;                  /**< General information of the measurement */
        double utc_seconds;             /**< UTC seconds from midnight */
        double lat;                     /**< Latitude of the position */
        double lon;                     /**< Longitude of the position */
        char lat_dir;                   /**< Direction of the latitude (N or S) */
        char lon_dir;                   /**< Direction of the longitude (W or E) */
        uint32_t gpq_quality;           /**< GPS quality indicator */
        uint32_t num_sats;              /**< Number of satellites in view */
        float hdop;                     /**< Relative accuracy of the horizontal position */
        float altitude;                 /**< Altitude */
        std::string altitude_units;     /**< Units of antenna altitude (e.g. M for meters) */
        float ondulation;               /**< Geoidal separation */
        std::string ondulation_units;   /**< Units of geoidal separation (e.g. M for meters) */
        uint32_t diff_age;              /**< Age of differential GPS data (seconds) */
        std::string station_id;         /**< Differential reference station ID */

        using Ptr = std::shared_ptr<struct GpggaMSG_tag>;               /**< Pointer to a GpggaMSG variable */
        using ConstPtr = std::shared_ptr<const struct GpggaMSG_tag>;    /**< Pointer to a constant GpggaMSG variable */
    } GpggaMSG;

/**
 * @brief Contains information about a GPGSA NMEA message (Global Positioning System DOP and active satellites), used to
 * represent the ID's of satellites which are used for the position fix.
 */
    typedef struct GpgsaMSG_tag {
        Header header;                  /**< General information of the measurement */
        char auto_manual_mode;          /**< Mode (M for manual, A for automatic) */
        uint8_t fix_mode;               /**< Mode fix (1 for no fix, 2 for 2D and 3 for 3D */
        std::vector<uint8_t> sv_ids;    /**< List of satellites used in the fix */
        float pdop;                     /**< Relative accuracy of the position */
        float hdop;                     /**< Relative accuracy of the horizontal position */
        float vdop;                     /**< Relative accuracy of the vertical position */

        using Ptr = std::shared_ptr<struct GpgsaMSG_tag>;               /**< Pointer to a GpgsaMSG variable */
        using ConstPtr = std::shared_ptr<const struct GpgsaMSG_tag>;    /**< Pointer to a constant GpgsaMSG variable */
    } GpgsaMSG;

/**
 * @brief Satellite data structure used in GPGSV NEMA messages.
 */
    typedef struct {
        uint8_t prn;        /**< Number of the satellite (GPS, SBAS or GLO) */
        uint8_t elevation;  /**< Elevation in degrees (max 90) */
        uint16_t azimuth;   /**< Azimuth, True North degrees (0 to 359) */
        int8_t snr;         /**< SNR in dB, -1 when not tracking */
    } GpgsvSatellite;

/**
 * @brief Contains information about a GPGSV NMEA message (Global Positioning System Satellites in view), used to describe
 * the informations associated to the satellites in view.
 */
    typedef struct GpgsvMSG_tag {
        Header header;                              /**< General information of the measurement */
        uint8_t n_msgs;                             /**< Number of messages in this sequence */
        uint8_t msg_number;                         /**< This message number in its sequence */
        uint8_t n_satellites;                       /**< Number of satellites currently visible */
        std::vector<GpgsvSatellite> satellites;  /**< Up to 4 satellites are described in each message */

        using Ptr = std::shared_ptr<struct GpgsvMSG_tag>;               /**< Pointer to a GpgsvMSG variable */
        using ConstPtr = std::shared_ptr<const struct GpgsvMSG_tag>;    /**< Pointer to a constant GpgsaMSG variable */
    } GpgsvMSG;

/**
 * @brief Contains information about a GPRMC NMEA message (Global Positioning System Recommended Minimum Specific GNSS Data),
 * containing time, date, position, course and speed provided by a GNSS navigation receiver.
 */
    typedef struct GprmcMSG_tag {
        Header header;         /**< General information of the measurement */
        double utc_seconds;    /**< UTC seconds from midnight */
        char status;           /**< Status (A for valid data, V for receiver warning) */
        double lat;            /**< Latitude of the position */
        double lon;            /**< Longitude of the position */
        char lat_dir;          /**< Direction of the latitude (N or S) */
        char lon_dir;          /**< Direction of the longitude (W or E) */
        float speed;           /**< Speed over ground in knots */
        float track;           /**< Course over ground in degrees */
        std::string date;      /**< UTC date of position fix */
        float mag_var;         /**< Magnetic variation degrees */
        char mag_var_dir;      /**< Direction of the magnetic variation (W or E) */
        char mode_indicator;   /**< Mode (N not valid, A autonomous, D differential or E estimated) */

        using Ptr = std::shared_ptr<struct GprmcMSG_tag>;               /**< Pointer to a GprmcMSG variable */
        using ConstPtr = std::shared_ptr<const struct GprmcMSG_tag>;    /**< Pointer to a constant GprmcMSG variable */
    } GprmcMSG;

/**
 * @brief Contains information about a Sentence NMEA message.
 */
    typedef struct Sentence_tag {
        Header header;          /**< General information of the measurement */
        std::string sentence;   /**< String describing the satellite measurement */

        using Ptr = std::shared_ptr<struct Sentence_tag>;               /**< Pointer to a SentenceMSG variable */
        using ConstPtr = std::shared_ptr<const struct Sentence_tag>;    /**< Pointer to a constant SentenceMSG variable */
    } SentenceMSG;
}

/**
 * @brief Status of a NavSatFix message.
 */
typedef struct{
    int8_t STATUS_NO_FIX = -1;      /**< Unable to fix the position */
    int8_t STATUS_FIX = 0;          /**< Unaugmented fix */
    int8_t STATUS_SBAS_FIX = 1;     /**< Fix with satellite-based augmentation */
    int8_t STATUS_GBAS_FIX = 2;     /**< Fix with ground-based augmentation */
    int8_t status;                  /**< Position fix status */
    uint16_t SERVICE_GPS = 1;       /**< GPS service identifier */
    uint16_t SERVICE_GLONASS = 2;   /**< GLONASS service identifier */
    uint16_t SERVICE_COMPASS = 4;   /**< COMPASS service identifier */
    uint16_t SERVICE_GALILEO = 8;   /**< GALILEO service identifier */
    uint16_t service;               /**< Which service has been used */
} NavSatStatus;

/**
 * @brief Navigation Satellite fix for any Global Navigation Satellite System.
 */
typedef struct NavSatFixMSG_tag{
    Header header;                              /**< General information of the measurement */
    NavSatStatus status;                        /**< Satellite fix status information */
    double lat;                                 /**< Latitude in degrees, positive if above the equator, negative otherwise */
    double lon;                                 /**< Longitude in degrees, positive if east of the prime meridian, negative otherwise */
    double alt;                                 /**< Altitude in meters, positive if above the WGS 84 ellipsoid */
    double position_covariance[9];              /**< Position covariance in squared meters, components are ENU */
    uint8_t COVARIANCE_TYPE_UNKNOWN = 0;        /**< Unknown covariance */
    uint8_t COVARIANCE_TYPE_APPROXIMATED = 1;   /**< Approximated covariance, using dilution of precision (accuracy) */
    uint8_t COVARIANCE_TYPE_DIAGONAL_KNOWN = 2; /**< The GPS provides the variance of each measurement, to be used in the diagonal */
    uint8_t COVARIANCE_TYPE_KNOWN = 3;          /**< Known covariance */
    uint8_t position_covariance_type;           /**< Type of covariance used */

    using Ptr = std::shared_ptr<struct NavSatFixMSG_tag>;               /**< Pointer to a NavSatFixMSG variable */
    using ConstPtr = std::shared_ptr<const struct NavSatFixMSG_tag>;    /**< Pointer to a constant NavSatFixMSG variable */
} NavSatFixMSG;

/**
 * @brief Geographic point message, with general information.
 */
typedef struct GeoPointStampedMSG_tag{
    Header header;    /**< General information of the measurement */
    double lat;       /**< Latitude in degrees, positive if above the equator, negative otherwise */
    double lon;       /**< Longitude in degrees, positive if east of the prime meridian, negative otherwise */
    double alt;       /**< Altitude in meters, positive if above the WGS 84 ellipsoid */

    using Ptr = std::shared_ptr<struct GeoPointStampedMSG_tag>;             /**< Pointer to a GeoPointStampedMSG variable */
    using ConstPtr = std::shared_ptr<const struct GeoPointStampedMSG_tag>;  /**< Pointer to a const GeoPointStampedMSG variable */
} GeoPointStampedMSG;


#endif //ARTSLAM_SLAM_TYPES_H
