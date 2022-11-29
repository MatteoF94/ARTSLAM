#include "tracker.h"
#include <chrono>
#include <utility>

using namespace artslam::laser3d;

// Class constructor
Tracker::Tracker(pcl::Registration<Point3I, Point3I>::Ptr registration_method) {
    std::stringstream msg;

    if(configuration_.verbose_) {
        msg << "[Tracker] Creating and configuring the tracker\n";
        std::cout << msg.str();
        msg.str("");
    }

    height_filter_.setFilterFieldName(configuration_.filter_axis_);
    height_filter_.setFilterLimits(configuration_.min_height_, FLT_MAX);
    tracker_dispatcher_ = std::make_unique<Dispatcher>("TrackerDispatcher", 1);
    registration_method_ = std::move(registration_method);

    if(configuration_.verbose_) {
        msg << "[Tracker] Finished creating and configuring the tracker\n";
        std::cout << msg.str();
    }
}

Tracker::Tracker(const Configuration &configuration, pcl::Registration<Point3I, Point3I>::Ptr registration_method) {
    std::stringstream msg;
    configuration_.verbose_ = configuration.verbose_;

    if(configuration_.verbose_) {
        msg << "[Tracker] Creating and configuring the tracker\n";
        std::cout << msg.str();
    }

    configuration_.keyframe_delta_trans_ = configuration.keyframe_delta_trans_;
    configuration_.keyframe_delta_angle_ = configuration.keyframe_delta_angle_;
    configuration_.keyframe_delta_time_ = configuration.keyframe_delta_time_;
    configuration_.use_height_filter_ = configuration.use_height_filter_;
    configuration_.filter_axis_ = configuration.filter_axis_;
    configuration_.min_height_ = configuration.min_height_;
    configuration_.use_prior_odometry_ = configuration.use_prior_odometry_;
    configuration_.prior_odometry_delta_trans_ = configuration.prior_odometry_delta_trans_;
    configuration_.prior_odometry_delta_angle_ = configuration.prior_odometry_delta_angle_;
    configuration_.max_odometry_delta_time_ = configuration.max_odometry_delta_time_;
    configuration_.initial_orientation_ = configuration.initial_orientation_;
    configuration_.initial_translation_ = configuration.initial_translation_,
    configuration_.utm_compensated_ = configuration.utm_compensated_;
    configuration_.verbose_ = configuration.verbose_;

    height_filter_.setFilterFieldName(configuration_.filter_axis_);
    height_filter_.setFilterLimits(configuration_.min_height_, FLT_MAX);
    tracker_dispatcher_ = std::make_unique<Dispatcher>("TrackerDispatcher", 1);
    registration_method_ = std::move(registration_method);

    if(configuration_.verbose_) {
        msg.str("");
        msg << "[Tracker] Finished creating and configuring the tracker\n";
        std::cout << msg.str();
    }
}

// Signals that a new filtered point cloud has been received
void Tracker::update_filtered_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) {
    tracker_dispatcher_->dispatch([this, pointcloud]{track(pointcloud);});
}

// Signals that a new odometry measurement has been received
void Tracker::update_prior_odometry_observer(OdometryStamped3D_MSG::ConstPtr odometry3d_msg) {
    std::lock_guard<std::mutex> odometry_lock(prior_odometry_mutex_);
    odometry3d_msgs_.emplace_back(odometry3d_msg);
}

// Registers an object waiting for keyframes
void Tracker::register_keyframe_observer(KeyframeObserver *keyframe_observer) {
    keyframe_observers_.emplace_back(keyframe_observer);
}

// Removes an object waiting for keyframes
void Tracker::remove_keyframe_observer(KeyframeObserver *keyframe_observer) {
    auto iterator = std::find(keyframe_observers_.begin(), keyframe_observers_.end(), keyframe_observer);

    if(iterator != keyframe_observers_.end()) {
        keyframe_observers_.erase(iterator);
    }
}

// Notifies all the objects waiting for keyframes
void Tracker::notify_keyframe_laser3d_observers(const KeyframeLaser3D::Ptr& keyframe) {
    for(KeyframeObserver* observer : keyframe_observers_) {
        observer->update_keyframe_observer(keyframe);
    }
}

// Registers an object waiting for odometries
void Tracker::register_odometry_observer(OdometryObserver* odometry_observer) {
    odometry_observers_.emplace_back(odometry_observer);
}

// Removes an object waiting for odometries
void Tracker::remove_odometry_observer(OdometryObserver* odometry_observer) {
    auto iterator = std::find(odometry_observers_.begin(), odometry_observers_.end(), odometry_observer);

    if(iterator != odometry_observers_.end()) {
        odometry_observers_.erase(iterator);
    }
}

// Notifies all the objects waiting for filtered point clouds
void Tracker::notify_odometry_observers(const OdometryStamped3D_MSG::ConstPtr& odometry3d_msg) {
    for(OdometryObserver* observer : odometry_observers_) {
        observer->update_odometry_observer(odometry3d_msg);
    }
}

void Tracker::track(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) {
    std::stringstream msg;
    auto start = std::chrono::high_resolution_clock::now();

    if(configuration_.verbose_) {
        msg << "[Tracker] Tracking cloud with (id, timestamp): (" << pointcloud->header.seq << ", " << pointcloud->header.stamp
            << ")\n";
        std::cout << msg.str();
        msg.str("");
    }

    pcl::PointCloud<Point3I>::Ptr filtered_pointcloud(new pcl::PointCloud<Point3I>());
    if(configuration_.use_height_filter_) {
        height_filter_.setInputCloud(pointcloud);
        height_filter_.filter(*filtered_pointcloud);
    } else {
        // this is costly, so it should be avoided aways
        filtered_pointcloud = pointcloud->makeShared();
    }
    filtered_pointcloud->header = pointcloud->header;

    Eigen::Matrix4f pose = match(filtered_pointcloud);
    if(skip_mode_) {
        if(configuration_.verbose_) {
            msg << "[Tracker] Skipping point cloud received, with (id, timestamp): (" << pointcloud->header.seq << ", " << pointcloud->header.stamp
                << ")\n";
            std::cout << msg.str();
            msg.str("");
        }
        return;
    }

    if(first_keyframe_) {
        first_keyframe_ = false;

        KeyframeLaser3D::Ptr keyframe = create_keyframe();

        // notify all keyframe observers
        notify_keyframe_laser3d_observers(keyframe);

        // notify all odometry observers
        OdometryStamped3D_MSG::Ptr odometry3d_msg(new OdometryStamped3D_MSG());
        odometry3d_msg->header_.timestamp_ = pointcloud->header.stamp;
        odometry3d_msg->header_.frame_id_ = pointcloud->header.frame_id;
        odometry3d_msg->header_.frame_id_ = pointcloud->header.frame_id;
        notify_odometry_observers(odometry3d_msg);
    } else {
        if(is_odometry_updated(pointcloud, pose)) {
            enable_prior_odometry_ = true;
            KeyframeLaser3D::Ptr keyframe = create_keyframe();

            // notify all keyframe observers
            notify_keyframe_laser3d_observers(keyframe);

            // notify all odometry observers
            OdometryStamped3D_MSG::Ptr odometry3d_msg(new OdometryStamped3D_MSG());
            odometry3d_msg->header_.timestamp_ = pointcloud->header.stamp;
            odometry3d_msg->header_.frame_id_ = pointcloud->header.frame_id;
            odometry3d_msg->header_.frame_id_ = pointcloud->header.frame_id;
            notify_odometry_observers(odometry3d_msg);
        }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    total_time_ += duration.count();
    count_++;

    if(configuration_.verbose_) {
        msg << "[Tracker] Total time and count: " << total_time_ << ", " << count_ << "\n";
        std::cout << msg.str();
    }
}

EigMatrix4f Tracker::match(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud) {
    auto start = std::chrono::high_resolution_clock::now();
    std::stringstream msg;

    // immediately check if this is the first keyframe, i.e., the first received point cloud
    if(first_frame_) {
        if(configuration_.verbose_) {
            msg << "[Tracker] First point cloud received, with (id, timestamp): (" << pointcloud->header.seq << ", " << pointcloud->header.stamp
                << ")\n";
            std::cout << msg.str();
            msg.str("");
        }

        first_frame_ = false;

        keyframe_pose_.setIdentity();
        EigMatrix3f rotation = configuration_.initial_orientation_.normalized().toRotationMatrix();
        keyframe_pose_.block<3,3>(0,0) = rotation;
        keyframe_pose_.block<3,1>(0,3) = configuration_.initial_translation_;
        prev_transformation_.setIdentity();
        keyframe_timestamp_ = pointcloud->header.stamp;
        keyframe_pointcloud_ = pointcloud;
        registration_method_->setInputTarget(keyframe_pointcloud_);
        prior_odometry_transformation_.setIdentity();
        return EigMatrix4f::Identity();
    }

    // find the motion guess to use for scan matching
    EigMatrix4f motion_guess;
    if(!configuration_.use_prior_odometry_ || !has_prior_odometry_guess_(pointcloud->header.stamp)) {
        motion_guess = prev_transformation_;
    } else {
        motion_guess = prior_odometry_transformation_;
        prev_transformation_ = prior_odometry_transformation_;  // very important!
    }

    // if the odometry measurements allow to skip scan matching, do it
    if(skip_mode_) {
        return EigMatrix4f::Identity();
    }

    // point cloud matching phase
    registration_method_->setInputSource(pointcloud);
    pcl::PointCloud<Point3I>::Ptr aligned_pointcloud(new pcl::PointCloud<Point3I>());
    registration_method_->align(*aligned_pointcloud, motion_guess);

    if(!registration_method_->hasConverged()) {
        if(configuration_.verbose_) {
            msg << "[Tracker] Matching with cloud with (id, timestamp): (" << pointcloud->header.seq << ", " << pointcloud->header.stamp
                << "), has not converged; ignoring this frame\n";
            std::cout << msg.str();
            msg.str("");
        }

        // assumes a constant motion model
        return keyframe_pose_ * prev_transformation_;
    }

    EigMatrix4f transformation = registration_method_->getFinalTransformation();
    current_fitness_ = registration_method_->getFitnessScore();
    EigMatrix4f odometry = keyframe_pose_ * transformation;

    prev_transformation_ = transformation;
    return odometry;
}

bool Tracker::has_prior_odometry_guess_(const uint64_t timestamp) {
    std::unique_lock<std::mutex> odometry_lock(prior_odometry_mutex_);

    // if there are no odometries to check, return immediately
    if(odometry3d_msgs_.empty() || !enable_prior_odometry_) {
        return false;
    }

    OdometryStamped3D_MSG::ConstPtr odometry3d_msg = odometry3d_msgs_.front();

    // TODO an alternative solution would be to check the closest in time (even after the pointcloud timestamp)
    // search for the closest odometry datum, in time, to the point cloud
    auto loc = odometry3d_msgs_.begin();
    for(; loc != odometry3d_msgs_.end(); loc++) {
        odometry3d_msg = (*loc);
        if((*loc)->header_.timestamp_ > timestamp) {
            break;
        }
    }

    // remove old and useless odometry data and unlock the mutex
    odometry3d_msgs_.erase(odometry3d_msgs_.begin(), loc);
    odometry_lock.unlock();

    // lastly, check if there is a too large gap between the current odometry and the keyframe
    if(timestamp - odometry3d_msg->header_.timestamp_ > configuration_.max_odometry_delta_time_) {
        return false;
    }

    // check whether the odometry information allows to skip scan matching later
    prior_odometry_transformation_ = keyframe_pose_.inverse() * odometry3d_msg->odometry_;
    double delta_translation = prior_odometry_transformation_.block<3,1>(0,3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(prior_odometry_transformation_.block<3,3>(0,0)).w());
    if(delta_translation > configuration_.prior_odometry_delta_trans_ || delta_angle > configuration_.prior_odometry_delta_angle_) {
        skip_mode_ = false;
        enable_prior_odometry_ = false;
    } else {
        skip_mode_ = true;
    }

    return true;
}

// Creates a new keyframe
KeyframeLaser3D::Ptr Tracker::create_keyframe() {
    EigIsometry3d odometry;
    odometry.matrix() = keyframe_pose_.cast<double>();
    KeyframeLaser3D::Ptr keyframe(new KeyframeLaser3D(keyframe_timestamp_, accumulated_distance_, current_fitness_, odometry, keyframe_pointcloud_));
    keyframe->utm_compensated_ = configuration_.utm_compensated_;
    return keyframe;
}

bool Tracker::is_odometry_updated(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud, const EigMatrix4f& odometry) {
    double delta_translation = prev_transformation_.block<3,1>(0,3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(prev_transformation_.block<3,3>(0,0)).w());
    uint64_t delta_time = pointcloud->header.stamp - keyframe_timestamp_;
    if(delta_translation > configuration_.keyframe_delta_trans_ || delta_angle > configuration_.keyframe_delta_angle_ || delta_time > configuration_.keyframe_delta_time_) {
        keyframe_pointcloud_ = pointcloud;
        registration_method_->setInputTarget(keyframe_pointcloud_);
        keyframe_pose_ = odometry;
        keyframe_timestamp_ = pointcloud->header.stamp;
        prev_transformation_.setIdentity();
        accumulated_distance_ += delta_translation;
        prior_odometry_transformation_.setIdentity();

        return true;
    }

    return false;
}

void Tracker::store_guesses(const std::vector<EigIsometry3f> &guesses) {
    guesses_ = guesses;
}
