/** @file keyframe_observer.h
 * @brief Declaration of class KeyframeObserver
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_KEYFRAME_OBSERVER_H
#define ARTSLAM_KEYFRAME_OBSERVER_H


#include <keyframe.h>

/**
 * @class KeyframeObserver
 * @brief Interface for objects waiting for keyframes, e.g. loop detectors.
 */
class KeyframeObserver {
public:
    /**
     * @brief Virtual update method.
     * @param keyframe Pointer to the waited keyframe.
     */
    virtual void update(const Keyframe::Ptr& keyframe) = 0;
};


#endif //ARTSLAM_KEYFRAME_OBSERVER_H
