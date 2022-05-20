#ifndef ARTSLAM_LASER_3D_LOOP_DETECTOR_H
#define ARTSLAM_LASER_3D_LOOP_DETECTOR_H

#include "keyframe_laser_3d.h"
#include "registration.h"
#include "scancontext/scancontext.h"

namespace artslam::laser3d {
    struct Loop {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Ptr = std::shared_ptr<Loop>;
        using ConstPtr = std::shared_ptr<const Loop>;

        Loop(const KeyframeLaser3D::Ptr& key1, const KeyframeLaser3D::Ptr& key2, const EigMatrix4f& relpose, double fitness) : keyframe_source_(key1), keyframe_target_(key2), relative_pose_(relpose), reliability_(fitness) {}

    public:
        KeyframeLaser3D::Ptr keyframe_source_;
        KeyframeLaser3D::Ptr keyframe_target_;
        EigMatrix4f relative_pose_;
        double reliability_ = 0;
    };

    class LoopDetector {
    public:
        struct Configuration {
            double distance_threshold_ = 30.0;                  // estimated distance between keyframes consisting a loop must be less than this distance
            double accumulated_distance_threshold_ = 25.0;      // minimum travelled distance between keyframes to be considered a candidate pair for loop closure
            double distance_from_last_edge_threshold_ = 15.0;   // a new loop edge must be far from the last one at least this distance
            bool use_scan_context_ = false;                      // whether to use scan context
            int best_k_contexts_ = 1;                           // number of candidates derived from scan context
            double fitness_score_max_range_ = std::numeric_limits<double>::max();   // maximum allowable distance between corresponding points
            double fitness_score_threshold_ = 2.5;                                  // threshold for scan matching
            bool verbose_ = true;                              // whether the class should be verbose
        };

        explicit LoopDetector(pcl::Registration<Point3I, Point3I>::Ptr registration_method);

        LoopDetector(const Configuration& configuration, pcl::Registration<Point3I, Point3I>::Ptr registration_method);

        void make_scancontext(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        std::vector<Loop::Ptr> detect(const std::vector<KeyframeLaser3D::Ptr>& keyframes, const std::deque<KeyframeLaser3D::Ptr>& new_keyframes);

        // testing purposes
        uint64_t total_time_ = 0;
        uint32_t count_ = 0;

    private:
        // Finds loop closure candidates within a certain distance range, and not too close to the last loops
        std::vector<KeyframeLaser3D::Ptr> find_candidates(const std::vector<KeyframeLaser3D::Ptr>& keyframes, const KeyframeLaser3D::Ptr& new_keyframe) const;

        // Finds loop closure candidates following the scan context procedure
        std::vector<KeyframeLaser3D::Ptr> find_candidates_sc(const std::vector<KeyframeLaser3D::Ptr>& keyframes, const KeyframeLaser3D::Ptr& new_keyframe);

        // Finds the best candidate for loop closure, if it exists
        Loop::Ptr matching(const std::vector<KeyframeLaser3D::Ptr>& candidate_keyframes, const KeyframeLaser3D::Ptr& new_keyframe);

        // ----------------------------------------------------------------------------------
        // ---------------------------- PARAMETERS AND VARIABLES ----------------------------
        // ----------------------------------------------------------------------------------
        Configuration configuration_;
        double last_edge_accum_distance = 0.0;
        pcl::Registration<Point3I,Point3I>::Ptr registration_method_;
        SCManager scan_context_;

    };
}


#endif //ARTSLAM_LASER_3D_LOOP_DETECTOR_H
