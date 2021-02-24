/** @file imu_observer.h
 * @brief Declaration of class ImuObserver
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_IMU_OBSERVER_H
#define ARTSLAM_IMU_OBSERVER_H


#include <slam_types.h>

/**
 * @class ImuObserver
 * @brief Interface for objects waiting for IMU data, e.g. prefilterer, backend or UKF.
 */
class ImuObserver {
public:
    /**
     * @brief Virtual update method.
     * @param imu_msg_constptr The IMU data used to update the concrete object.
     */
    virtual void update(const ImuMSG::ConstPtr& imu_msg_constptr) = 0;
};


#endif //ARTSLAM_IMU_OBSERVER_H
