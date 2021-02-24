/** @file loop.h
 * @brief Declaration of class Loop
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_LOOP_H
#define ARTSLAM_LOOP_H


#include <keyframe.h>
#include <Eigen/src/Core/util/Memory.h>
#include <memory>

/**
 * @class Loop
 * @brief This class represents a loop in the trajectory of the robot.
 */
class Loop {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Pointer to the object Loop.
     */
    using Ptr = std::shared_ptr<Loop>;

    /**
     * @brief Class constructor, with parameter.
     * @param key_1 First keyframe of the loop.
     * @param key_2 Second keyframe of the loop.
     * @param relative_pose Relative motion between the two keyframes.
     */
    Loop(const Keyframe::Ptr& key_1, const Keyframe::Ptr& key_2, const Eigen::Matrix4f& relative_pose);

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    Keyframe::Ptr keyframe_1;       /**< First keyframe of the loop */
    Keyframe::Ptr keyframe_2;       /**< Second keyframe of the loop */
    Eigen::Matrix4f relative_pose;  /**< Relative motion between the two keyframes */
};


#endif //ARTSLAM_LOOP_H
