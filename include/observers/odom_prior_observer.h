/** @file odom_observer.h
 * @brief Declaration of class OdomPriorObserver
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_ODOM_PRIOR_OBSERVER_H
#define ARTSLAM_ODOM_PRIOR_OBSERVER_H


#include <slam_types.h>

/**
 * @class OdomObserver
 * @brief Interface for objects waiting for odometries, e.g. loop detectors, optimizers and visualizer.
 */
class OdomPriorObserver {
public:
    /**
     * @brief Virtual update method.
     * @param header The header associated to prior or predicted odometry (timestamp in nanoseconds, sequence and frame ID).
     * @param odom Predicted or prior odometry.
     */
    virtual void update(const Header& header, const Eigen::Matrix4f& odom) = 0;
};


#endif //ARTSLAM_ODOM_PRIOR_OBSERVER_H
