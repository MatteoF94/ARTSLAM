/** @file loop.cpp
 * @brief Definition of class Loop
 * @author Matteo Frosi
 */

#include <loop.h>

/**
 * @brief Pointer to the object Loop.
 */
Loop::Loop(const Keyframe::Ptr &key_1, const Keyframe::Ptr& key_2, const Eigen::Matrix4f &relative_pose) {
    this->keyframe_1 = key_1;
    this->keyframe_2 = key_2;
    this->relative_pose = relative_pose;
}