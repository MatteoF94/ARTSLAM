/** @file floor_coeffs_observer.h
 * @brief Declaration of class FloorCoeffsObserver
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_FLOOR_COEFFS_OBSERVER_H
#define ARTSLAM_FLOOR_COEFFS_OBSERVER_H


#include <slam_types.h>

/**
 * @class FloorCoeffsObserver
 * @brief Interface for objects waiting for plane normal coefficients, e.g. visualizers or loop detectors.
 */
class FloorCoeffsObserver {
public:
    /**
     * @brief Virtual update method.
     * @param floor_coeffs_msg_constptr The plane normal coefficients used to update the concrete object.
     */
    virtual void update(const FloorCoeffsMSG::ConstPtr& floor_coeffs_msg_constptr) = 0;
};


#endif //ARTSLAM_FLOOR_COEFFS_OBSERVER_H
