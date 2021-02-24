/** @file gps_observer.h
 * @brief Declaration of class GpsObserver
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_GPS_OBSERVER_H
#define ARTSLAM_GPS_OBSERVER_H


#include <slam_types.h>

/**
 * @class GpsObserver
 * @brief Interface for objects waiting for GPS data, e.g. backend or UKF.
 */
class GpsObserver {
public:
    /**
     * @brief Virtual update method.
     * @param gps_msg_constptr The GPS data used to update the concrete object.
     */
    virtual void update(const GeoPointStampedMSG::ConstPtr& gps_msg_constptr) = 0;
};


#endif //ARTSLAM_GPS_OBSERVER_H
