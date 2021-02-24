/** @file loop_detector.cpp
 * @brief Definition of class LoopDetector
 * @author Matteo Frosi
 */

#include <loop_detector.h>
#include <fstream>

/* ------------------ */
/* Class constructor. */
/* ------------------ */
LoopDetector::LoopDetector() {
    LoopDetectorConfig default_config;
    configure_loop_detector(&default_config);
}

/* ---------------------------------- */
/* Class constructor, with parameter. */
/* ---------------------------------- */
LoopDetector::LoopDetector(const LoopDetectorConfig *const config_ptr) {
    configure_loop_detector(config_ptr);
}

/* ----------------------------------------------------------------- */
/* Configures the loop detector object using a configuration object. */
/* ----------------------------------------------------------------- */
void LoopDetector::configure_loop_detector(const LoopDetectorConfig *const config_ptr) {
    std::ostringstream to_print;

    to_print << "[LoopDetector][configure_loop_detector] creating and configuring the loop detector\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    this->distance_thresh = config_ptr->distance_thresh;
    this->accum_distance_thresh = config_ptr->accum_distance_thresh;
    this->distance_from_last_edge_thresh = config_ptr->distance_from_last_edge_thresh;

    //this->fitness_score_max_range = config_ptr->fitness_score_max_range;
    this->fitness_score_max_range = std::numeric_limits<double>::max();
    this->fitness_score_thresh = config_ptr->fitness_score_thresh;

    this->last_edge_accum_distance = 0.0;

    to_print << "[LoopDetector][configure_loop_detector] DISTANCE_THRESH: " << this->distance_thresh << "\n";
    to_print << "[LoopDetector][configure_loop_detector] ACCUM_DISTANCE_THRESH: " << this->accum_distance_thresh << "\n";
    to_print << "[LoopDetector][configure_loop_detector] DISTANCE_FROM_LAST_EDGE_THRESH: " << this->distance_from_last_edge_thresh << "\n";
    to_print << "[LoopDetector][configure_loop_detector] FITNESS_SCORE_MAX_RANGE: " << this->fitness_score_max_range << "\n";
    to_print << "[LoopDetector][configure_loop_detector] FITNESS_SCORE_THRESH: " << this->fitness_score_thresh << "\n";
    to_print << "[LoopDetector][configure_loop_detector] finished configuring the loop detector\n";
    std::cout << to_print.str();
}

/* ---------------------------------------------------- */
/* Sets the registration method used for scan matching. */
/* ---------------------------------------------------- */
void LoopDetector::set_registration(Registration *registration_ptr) {
    this->registration = registration_ptr;
    // TODO the parameter below is useless in parallel mode, remove it
    this->registration_method = this->registration->select_registration_method();
}

/* -------------------------------------------------------- */
/* Loops are detected and added to the pose graph as edges. */
/* -------------------------------------------------------- */
std::vector<Loop::Ptr> LoopDetector::detect(const std::vector <Keyframe::Ptr>& keyframes, const std::deque <Keyframe::Ptr>& new_keyframes) {
    std::ostringstream to_print;

    to_print << "[LoopDetector][detect] finding loop closures\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    std::vector<Loop::Ptr> detected_loops;

    /*#pragma omp parallel default(none) shared(keyframes, new_keyframes, to_print, std::cout, detected_loops)
    {
        std::vector<Loop::Ptr> private_loops;
        #pragma omp for nowait*/
        for(int i = 0; i < new_keyframes.size(); i++) {
            Keyframe::Ptr new_keyframe = new_keyframes[i];
            std::vector<Keyframe::Ptr> candidates = find_candidates(keyframes, new_keyframe);
            Loop::Ptr loop = matching(candidates, new_keyframe);

            if (loop) {
                to_print << "[LoopDetector][detect] found loop closure between keyframes: ["
                         << loop->keyframe_1->cloud->header.seq << " - " << loop->keyframe_2->cloud->header.seq
                         << "]\n";
                std::cout << to_print.str();
                to_print.str("");
                to_print.clear();
                detected_loops.emplace_back(loop);
                //private_loops.emplace_back(loop);
            }
        }

        /*#pragma omp critical
        detected_loops.insert(detected_loops.end(), private_loops.begin(), private_loops.end());
    }*/

    if(detected_loops.empty()) {
        to_print << "[LoopDetector][detect] no loop closure found\n";
    }

    to_print << "[LoopDetector][detect] finished finding loop closures\n";
    std::cout << to_print.str();
    return detected_loops;
}

/* -------------------------------------------------------------------------------------- */
/* Finds possible loop candidates, evaluating keyframes using their accumulated distance. */
/* -------------------------------------------------------------------------------------- */
std::vector<Keyframe::Ptr> LoopDetector::find_candidates(const std::vector <Keyframe::Ptr> &keyframes,
                                                         const Keyframe::Ptr& new_keyframe) const {
    // too close from the last loop edge
    if(new_keyframe->accum_distance - this->last_edge_accum_distance < this->distance_from_last_edge_thresh) {
        return std::vector<Keyframe::Ptr>();
    }

    std::vector<Keyframe::Ptr> candidates;
    candidates.reserve(32);

    for(const Keyframe::Ptr& keyframe : keyframes) {
        // small travelled distance between keyframes, skip this pair
        if(new_keyframe->accum_distance - keyframe->accum_distance < this->accum_distance_thresh) {
            continue;
        }

        if(new_keyframe->cloud->header.seq - keyframe->cloud->header.seq < 200) {
            continue;
        }

        const auto& pos_1 = keyframe->node->estimate().translation();
        const auto& pos_2 = new_keyframe->node->estimate().translation();

        // too large distance between keyframes
        double distance = (pos_1.head<2>() - pos_2.head<2>()).norm();
        if(distance > this->distance_thresh) {
            continue;
        }

        candidates.emplace_back(keyframe);
    }

    return candidates;
}

/* ------------------------------------------------------------------------------------------------- */
/* Performs scan matching between pairs of keyframes to find the best candidates for a loop closure. */
/* ------------------------------------------------------------------------------------------------- */
Loop::Ptr LoopDetector::matching(const std::vector <Keyframe::Ptr> &candidate_keyframes,
                                 const Keyframe::Ptr &new_keyframe) {
    // TODO parallelize also the matching in super MT environments
    // no candidates found
    if(candidate_keyframes.empty()) {
        return nullptr;
    }

    std::ofstream outp;
    outp.open("/home/matteo/Desktop/candidates.txt", std::ios_base::app);


    //pcl::Registration<PointT,PointT>::Ptr reg_method = this->registration->select_registration_method();
    //reg_method->setInputTarget(new_keyframe->cloud);
    this->registration_method->setInputTarget(new_keyframe->cloud);

    double best_fitness_score = std::numeric_limits<double>::max();
    Keyframe::Ptr best_matched;
    Eigen::Matrix4f relative_pose;

    // search for the best candidate through scan matching
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    for(const Keyframe::Ptr& candidate : candidate_keyframes) {
        outp << candidate->cloud->header.seq << " - " << new_keyframe->cloud->header.seq << std::endl;
        outp << candidate->accum_distance << " - " << new_keyframe->accum_distance << std::endl;
        outp << candidate->node->estimate().matrix() << " + " << new_keyframe->node->estimate().matrix() << std::endl << std::endl;
        //reg_method->setInputSource(candidate->cloud);
        this->registration_method->setInputSource(candidate->cloud);
        Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate();
        new_keyframe_estimate.linear() = Eigen::Quaterniond(new_keyframe_estimate.linear()).normalized().toRotationMatrix();
        Eigen::Isometry3d candidate_estimate = candidate->node->estimate();
        candidate_estimate.linear() = Eigen::Quaterniond(candidate_estimate.linear()).normalized().toRotationMatrix();
        Eigen::Matrix4f guess = (new_keyframe_estimate.inverse() * candidate_estimate).matrix().cast<float>();
        guess(2,3) = 0.0;
        //reg_method->align(*aligned, guess);
        this->registration_method->align(*aligned, guess);

        double fitness_score = this->registration_method->getFitnessScore(this->fitness_score_max_range);
        if(!this->registration_method->hasConverged() || fitness_score > best_fitness_score) {
            continue;
        }
        /*double fitness_score = reg_method->getFitnessScore(this->fitness_score_max_range);
        if(!reg_method->hasConverged() || fitness_score > best_fitness_score) {
            continue;
        }*/

        best_fitness_score = fitness_score;
        best_matched = candidate;
        relative_pose = this->registration_method->getFinalTransformation();
        //relative_pose = reg_method->getFinalTransformation();
    }

    // no loop found
    if(best_fitness_score > this->fitness_score_thresh) {
        return nullptr;
    }

    this->last_edge_accum_distance = new_keyframe->accum_distance;

    return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);
}
