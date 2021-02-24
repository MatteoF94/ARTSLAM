/** @file loop_detector.h
 * @brief Declaration of class LoopDetector
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_LOOP_DETECTOR_H
#define ARTSLAM_LOOP_DETECTOR_H


#include <slam_types.h>
#include <loop.h>
#include <registration.h>
#include <graph_handler.h>
#include <loop_detector_config.h>
#include <deque>
#include <pcl/registration/registration.h>

/**
 * @class LoopDetector
 * @brief This class is used to identify loop closures and compute the relative motion between the involved keyframes.
 */
class LoopDetector {
public:
    /**
     * @brief Class constructor.
     */
    LoopDetector();

    /**
     * @brief Class constructor, with parameter.
     * @param config_ptr Pointer to the configuration object for this class.
     */
    explicit LoopDetector(const LoopDetectorConfig* config_ptr);

    /**
     * @brief Sets the registration method used for scan matching.
     * @param registration Pointer to the registration method.
     */
    void set_registration(Registration* registration);

    /**
     * @brief Loops are detected and added to the pose graph as edges.
     * @param keyframes Keyframes collected up to this moment.
     * @param new_keyframes New keyframes to be matched against the old ones.
     * @return The detected loops.
     */
    std::vector<Loop::Ptr> detect(const std::vector<Keyframe::Ptr>& keyframes, const std::deque<Keyframe::Ptr>& new_keyframes);

private:
    // ---------------------------------------------------------------
    // -------------------- CONFIGURATION METHODS --------------------
    //----------------------------------------------------------------
    /**
     * @brief Configures the loop detector object using a configuration object.
     * @param config_ptr Pointer to the configuration object.
     */
    void configure_loop_detector(const LoopDetectorConfig *config_ptr);

    // ---------------------------------------------------------------
    // ---------------------- DETECTION METHODS ----------------------
    //----------------------------------------------------------------
    /**
     * @brief Finds possible loop candidates, evaluating keyframes using their accumulated distance.
     * @param keyframes Keyframes collected up to this moment.
     * @param new_keyframe New keyframes to be matched against the old ones.
     * @return Loop closure candidates.
     */
    std::vector<Keyframe::Ptr> find_candidates(const std::vector<Keyframe::Ptr>& keyframes, const Keyframe::Ptr& new_keyframe) const;

    /**
     * @brief Performs scan matching between pairs of keyframes to find the best candidates for a loop closure.
     * @param candidate_keyframes The candidate keyframes to be evaluated for a loop closure.
     * @param new_keyframe The new keyframe to match against the candidates.
     * @param graph_handler Object that handles the creation of elements in the pose graph.
     * @return The detected loop closure (can be none).
     */
    Loop::Ptr matching(const std::vector<Keyframe::Ptr>& candidate_keyframes, const Keyframe::Ptr& new_keyframe);

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    double distance_thresh;                 /**< Maximum estimated distance between keyframes to be considered candidates for a loop */
    double accum_distance_thresh;           /**< Minimum travelled distance between keyframes to be considered candidates for a loop */
    double distance_from_last_edge_thresh;  /**< Minimum distance between the new and the last loop edges */

    double fitness_score_max_range;         /**< Maximum allowable distance between corresponding points */
    double fitness_score_thresh;            /**< Threshold used for scan matching */

    double last_edge_accum_distance;        /**< Accumulated distance from the last loop edge */

    Registration* registration;                                    /**< Used to select the registration method */
    pcl::Registration<PointT,PointT>::Ptr registration_method;     /**< Method used to perform scan matching between point clouds */
};


#endif //ARTSLAM_LOOP_DETECTOR_H
