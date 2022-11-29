#include "loop_detector.h"
#include <chrono>
#include <utility>

using namespace artslam::laser3d;

LoopDetector::LoopDetector(pcl::Registration<Point3I, Point3I>::Ptr registration_method) {
    std::stringstream msg;

    if(configuration_.verbose_) {
        msg << "[LoopDetector] Creating and configuring the loop detector\n";
        std::cout << msg.str();
        msg.str("");
    }

    registration_method_ = std::move(registration_method);

    if(configuration_.verbose_) {
        msg << "[LoopDetector] Finished creating and configuring the loop detector\n";
        std::cout << msg.str();
    }
}

LoopDetector::LoopDetector(const Configuration &configuration,
                           pcl::Registration<Point3I, Point3I>::Ptr registration_method) {
    std::stringstream msg;

    configuration_.verbose_ = configuration.verbose_;
    if(configuration_.verbose_) {
        msg << "[LoopDetector] Creating and configuring the loop detector\n";
        std::cout << msg.str();
        msg.str("");
    }

    configuration_.distance_threshold_ = configuration.distance_threshold_;
    configuration_.accumulated_distance_threshold_ = configuration.accumulated_distance_threshold_;
    configuration_.distance_from_last_edge_threshold_ = configuration.distance_from_last_edge_threshold_;
    configuration_.use_scan_context_ = configuration.use_scan_context_;
    configuration_.best_k_contexts_ = configuration.best_k_contexts_;
    configuration_.fitness_score_max_range_ = configuration.fitness_score_max_range_;
    configuration_.fitness_score_threshold_ = configuration.fitness_score_threshold_;

    registration_method_ = std::move(registration_method);

    if(configuration_.verbose_) {
        msg << "[LoopDetector] Finished creating and configuring the loop detector\n";
        std::cout << msg.str();
    }
}

void LoopDetector::make_scancontext(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud) {
    scan_context_.makeAndSaveScancontextAndKeys(*pointcloud);
}

std::vector<Loop::Ptr> LoopDetector::detect(const std::vector<KeyframeLaser3D::Ptr> &keyframes,
                                            const std::deque<KeyframeLaser3D::Ptr> &new_keyframes) {
    std::stringstream msg;
    auto start = std::chrono::high_resolution_clock::now();

    std::vector<Loop::Ptr> detected_loops;
    for(const auto& new_keyframe : new_keyframes) {
        auto start1 = std::chrono::high_resolution_clock::now();
        // first step, filter by distance
        auto candidates = find_candidates(keyframes, new_keyframe);

        // second step, scan context
        if(configuration_.use_scan_context_ && candidates.size() >= 1 + configuration_.best_k_contexts_) {
            candidates = find_candidates_sc(candidates, new_keyframe);
        }

        // third step, scan matching
        auto loop = matching(candidates, new_keyframe);

        if(loop) {
            detected_loops.push_back(loop);
        }
        auto stop1 = std::chrono::high_resolution_clock::now();
        auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start1);

        /*if(configuration_.verbose_) {
            msg << "[LoopDetector] Key and time: " << new_keyframe->timestamp_ << ", " << duration1.count() << "\n";
            std::cout << msg.str();
            msg.str("");
        }*/
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    total_time_ += duration.count();
    count_++;

    /*if(configuration_.verbose_) {
        msg << "[LoopDetector] Total time and count: " << total_time_ << ", " << count_ << "\n";
        std::cout << msg.str();
    }*/

    return detected_loops;
}

std::vector<KeyframeLaser3D::Ptr> LoopDetector::find_candidates(const std::vector<KeyframeLaser3D::Ptr>& keyframes, const KeyframeLaser3D::Ptr& new_keyframe) const {
    // too close to the last registered loop edge
    if(new_keyframe->accumulated_distance_ - last_edge_accum_distance < configuration_.distance_from_last_edge_threshold_) {
        return {};
    }

    std::vector<KeyframeLaser3D::Ptr> candidates;
    candidates.reserve(32);

    for(const auto& k : keyframes) {
        // traveled distance between keyframes is too small
        if(new_keyframe->accumulated_distance_ - k->accumulated_distance_ < configuration_.accumulated_distance_threshold_) {
            continue;
        }

        const auto& pos1 = k->graph_node_->estimate().translation();
        const auto& pos2 = new_keyframe->graph_node_->estimate().translation();

        // estimated distance between keyframes is too small
        double dist = (pos1.head<2>() - pos2.head<2>()).norm();
        if(dist > configuration_.distance_threshold_) {
            continue;
        }

        candidates.push_back(k);
    }

    return candidates;
}

std::vector<KeyframeLaser3D::Ptr> LoopDetector::find_candidates_sc(const std::vector<KeyframeLaser3D::Ptr>& keyframes, const KeyframeLaser3D::Ptr& new_keyframe) {
    std::vector<KeyframeLaser3D::Ptr> candidates;

    std::vector<unsigned int> candidates_ids;
    for(const KeyframeLaser3D::Ptr& keyframe : keyframes)
        candidates_ids.emplace_back(keyframe->pointcloud_->header.seq);

    unsigned int query = new_keyframe->pointcloud_->header.seq;

    std::vector<std::pair<int, float>> new_candidates = scan_context_.detectLoopClosureID(candidates_ids, query, configuration_.best_k_contexts_);

    for(auto & pair : new_candidates) {
        if (pair.first != -1) {
            for (const KeyframeLaser3D::Ptr &keyframe: keyframes) {
                if (keyframe->pointcloud_->header.seq == pair.first) {
                    candidates.emplace_back(keyframe);
                }
            }
        }
    }

    return candidates;
}


Loop::Ptr LoopDetector::matching(const std::vector<KeyframeLaser3D::Ptr>& candidate_keyframes, const KeyframeLaser3D::Ptr& new_keyframe) {
    if(candidate_keyframes.empty()) {
        return nullptr;
    }

    std::stringstream msg;

    registration_method_->setInputTarget(new_keyframe->pointcloud_);

    double best_score = std::numeric_limits<double>::max();
    KeyframeLaser3D::Ptr best_matched;
    Eigen::Matrix4f relative_pose;
    double fitness = 0;

    if(configuration_.verbose_) {
        msg << "\n[LoopDetector] --- Loop detection ---\n";
        msg << "[LoopDetector] Number of candidates: " << candidate_keyframes.size() << "\n";
        msg << "[LoopDetector] Matching...\n";
        std::cout << msg.str();
        msg.str("");
    }

    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<Point3I>::Ptr aligned_pointcloud(new pcl::PointCloud<Point3I>());
    for(const auto& candidate : candidate_keyframes) {
        registration_method_->setInputSource(candidate->pointcloud_);
        msg << "[LoopDetector] Query - candidate: " << new_keyframe->pointcloud_->header.seq << " - " << candidate->pointcloud_->header.seq << "\n";
        std::cout << msg.str();
        msg.str("");

        // get the normalized rotation component of the target keyframe
        EigIsometry3d new_keyframe_estimate = new_keyframe->graph_node_->estimate();
        new_keyframe_estimate.linear() = EigQuaterniond(new_keyframe_estimate.linear()).normalized().toRotationMatrix();

        // get the normalized rotation component of the source (candidate) keyframe
        EigIsometry3d candidate_estimate = candidate->graph_node_->estimate();
        candidate_estimate.linear() = Eigen::Quaterniond(candidate_estimate.linear()).normalized().toRotationMatrix();

        // compute the motion guess, as transformation between the clouds, and perform scan matching
        EigMatrix4f guess = (new_keyframe_estimate.inverse() * candidate_estimate).matrix().cast<float>();
        guess(2, 3) = 0.0;  // set the "Z" component to 0 to have a 2D accurate guess
        //guess(1,3) = 0.0;
        //guess(0,3) = 0.0;
        registration_method_->align(*aligned_pointcloud, guess);

        double score = registration_method_->getFitnessScore(configuration_.fitness_score_max_range_);
        if(!registration_method_->hasConverged() || score > best_score) {
            continue;
        }

        best_score = score;
        best_matched = candidate;
        relative_pose = registration_method_->getFinalTransformation();
        fitness = registration_method_->getFitnessScore();

        if(configuration_.verbose_) {
            msg << "[LoopDetector] ID, ID, score, fitness: " << candidate->pointcloud_->header.seq << ", " <<
                                                                  new_keyframe->pointcloud_->header.seq << ", " <<
                                                                  score << ", " << fitness << "\n";
            std::cout << msg.str();
            msg.str("");
        }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    if(configuration_.verbose_) {
        msg << "[LoopDetector] Best score and time: " << best_score << ", " << duration.count() << "\n";
        std::cout << msg.str();
        msg.str("");
    }

    if(best_score > configuration_.fitness_score_threshold_) {
        if(configuration_.verbose_) {
            msg << "[LoopDetector] Loop not found...\n";
            std::cout << msg.str();
        }
        return nullptr;
    }

    if(configuration_.verbose_) {
        msg << "[LoopDetector] Loop found, between keyframes with ID: " << new_keyframe->pointcloud_->header.seq << ", " << best_matched->pointcloud_->header.seq << "\n";
        std::cout << msg.str();
        msg.str("");
    }
    //std::cout << "relpose: " << relative_pose.block<3, 1>(0, 3) << " - " << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0)).coeffs().transpose() << std::endl;

    last_edge_accum_distance = new_keyframe->accumulated_distance_;

    return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose, fitness);
}
