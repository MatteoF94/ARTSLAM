/** @file tracker.cpp
 * @brief Definition of class Tracker
 * @author Matteo Frosi
*/

#include <tracker.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <chrono>
#include <fstream>
#include <pcl/filters/passthrough.h>

/* ------------------ */
/* Class constructor. */
/* ------------------ */
Tracker::Tracker() {
    TrackerConfig default_config;
    configure_tracker(&default_config);
}

/* ---------------------------------- */
/* Class constructor, with parameter. */
/* ---------------------------------- */
Tracker::Tracker(const TrackerConfig *const config_ptr) {
    configure_tracker(config_ptr);
}

/* ----------------- */
/* Class destructor. */
/* ----------------- */
Tracker::~Tracker() {
    delete this->tracker_dqueue;
    delete this->prior_odom_insertion_dqueue;
}

/* ----------------------------------------------------------- */
/* Configures the tracker object using a configuration object. */
/* ----------------------------------------------------------- */
void Tracker::configure_tracker(const TrackerConfig *const config_ptr) {
    std::ostringstream to_print;

    to_print << "[Tracker][configure_tracker] creating and configuring the tracker\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    configure_downsample(config_ptr);

    this->keyframe_delta_trans = config_ptr->keyframe_delta_trans;
    this->keyframe_delta_angle = config_ptr->keyframe_delta_angle;
    this->keyframe_delta_time = config_ptr->keyframe_delta_time;

    this->transform_thresholding = config_ptr->transform_thresholding;
    this->max_acceptable_trans = config_ptr->max_acceptable_trans;
    this->max_acceptable_angle = config_ptr->max_acceptable_angle;

    this->is_first_frame = true;
    this->accumulated_distance = 0.0;
    this->current_fitness = 0.0;

    this->skip_by_ukf = false;

    this->prior_odom_delta_trans = config_ptr->prior_odom_delta_trans;
    this->prior_odom_delta_angle = config_ptr->prior_odom_delta_angle;

    this->total_time = 0;

    this->tracker_dqueue = new DispatchQueue("tracker queue", 1);
    this->prior_odom_insertion_dqueue = new DispatchQueue("prior odom insertion queue", 1);

    to_print << "[Tracker][configure_tracker] KEYFRAME_DELTA_TRANS: " << this->keyframe_delta_trans << "\n";
    to_print << "[Tracker][configure_tracker] KEYFRAME_DELTA_ANGLE: " << this->keyframe_delta_angle << "\n";
    to_print << "[Tracker][configure_tracker] KEYFRAME_DELTA_TIME: " << this->keyframe_delta_time << "\n";
    to_print << "[Tracker][configure_tracker] TRANSFORM_THRESHOLDING: " << this->transform_thresholding << "\n";
    to_print << "[Tracker][configure_tracker] MAX_ACCEPTABLE_TRANS: " << this->max_acceptable_trans << "\n";
    to_print << "[Tracker][configure_tracker] MAX_ACCEPTABLE_ANGLE: " << this->max_acceptable_angle << "\n";
    to_print << "[Tracker][configure_tracker] PRIOR_ODOM_DELTA_TRANS: " << this->prior_odom_delta_trans << "\n";
    to_print << "[Tracker][configure_tracker] PRIOR_ODOM_DELTA_ANGLE: " << this->prior_odom_delta_angle << "\n";
    to_print << "[Tracker][configure_tracker] finished creating and configurating the tracker\n";
    std::cout << to_print.str();
}

/* --------------------------------------------------------------- */
/* Configures the downsampling method used to filter point clouds. */
/* --------------------------------------------------------------- */
void Tracker::configure_downsample(const TrackerConfig *const config_ptr) {
    std::string downsample_method = config_ptr->downsample_method;
    float downsample_resolution = config_ptr->downsample_resolution;

    std::ostringstream to_print;

    if(downsample_method == "VOXELGRID") {
        to_print << "[Tracker][configure_downsample] downsample: VOXELGRID, res " << downsample_resolution << "\n";
        boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        this->downsample_filter = voxelgrid;
    } else if (downsample_method == "APPROX_VOXELGRID") {
        to_print << "[Tracker][configure_downsample] downsample: APPROX_VOXELGRID, res " << downsample_resolution << "\n";
        boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
        approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        this->downsample_filter = approx_voxelgrid;
    } else {
        if (downsample_method != "NONE") {
            to_print << "[Tracker][configure_downsample] downsample: NONE, no downsampling\n";
        }
    }

    std::cout << to_print.str();
}

/* ---------------------------------------------------- */
/* Sets the registration method used for scan matching. */
/* ---------------------------------------------------- */
void Tracker::set_registration(Registration *registration_ptr) {
    this->registration = registration_ptr;
    this->registration_method = this->registration->select_registration_method();
}

/* ------------------------------------------- */
/* Registers an object waiting for odometries. */
/* ------------------------------------------- */
void Tracker::register_odom_observer(OdomObserver *odom_observer_ptr_tmp) {
    this->odom_observers.emplace_back(odom_observer_ptr_tmp);
}

/* ----------------------------------------- */
/* Removes an object waiting for odometries. */
/* ----------------------------------------- */
void Tracker::remove_odom_observer(OdomObserver *odom_observer_ptr_tmp) {
    auto iterator = std::find(this->odom_observers.begin(), this->odom_observers.end(), odom_observer_ptr_tmp);

    if(iterator != this->odom_observers.end()) {
        this->odom_observers.erase(iterator);
    }
}

/* ------------------------------------------------------- */
/* Notifies all the observers with the estimated odometry. */
/* ------------------------------------------------------- */
void Tracker::notify_odom_observers(const Header& header, const Eigen::Matrix4f& odom) {
    for(OdomObserver *observer : this->odom_observers) {
        observer->update(header,odom);
    }
}

/* ------------------------------------------ */
/* Registers an object waiting for keyframes. */
/* ------------------------------------------ */
void Tracker::register_keyframe_observer(KeyframeObserver *kf_observer_ptr_tmp) {
    this->keyframe_observers.emplace_back(kf_observer_ptr_tmp);
}

/* ---------------------------------------- */
/* Removes an object waiting for keyframes. */
/* ---------------------------------------- */
void Tracker::remove_keyframe_observer(KeyframeObserver *kf_observer_ptr_tmp) {
    auto iterator = std::find(this->keyframe_observers.begin(), this->keyframe_observers.end(), kf_observer_ptr_tmp);

    if(iterator != this->keyframe_observers.end()) {
        this->keyframe_observers.erase(iterator);
    }
}

/* ------------------------------------------------------ */
/* Notifies all the observers with the computed keyframe. */
/* ------------------------------------------------------ */
void Tracker::notify_keyframe_observers(const Keyframe::Ptr& keyframe) {
    for(KeyframeObserver *observer : this->keyframe_observers) {
        observer->update(keyframe);
    }
}

/* ------------------------------------------------------------- */
/* Signals that a new filtered point cloud has been received. */
/* ------------------------------------------------------------- */
void Tracker::update(pcl::PointCloud<PointT>::ConstPtr src_cloud) {
    dispatch_tracking(src_cloud);
}

/* --------------------------------------------------- */
/* Signals that a prior odometry has been received. */
/* --------------------------------------------------- */
void Tracker::update(const Header& header, const Eigen::Matrix4f& odom) {
    dispatch_prior_odom_insertion(header, odom);
}

/* -------------------------------------------------------------------------------- */
/* Computes the tracking between the current point cloud and the previous keyframe. */
/* -------------------------------------------------------------------------------- */
void Tracker::compute_tracking(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    auto start = std::chrono::high_resolution_clock::now();
    int seq_id = cloud->header.seq;
    std::ostringstream to_print;

    to_print << "[Tracker][compute_tracking][seq:" << seq_id << "] tracking\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.6, FLT_MAX);
    pcl::PointCloud<PointT>::Ptr c1(new pcl::PointCloud<PointT>());
    pass.filter(*c1);

    Eigen::Matrix4f pose = matching(cloud->header.stamp, c1);
    if(this->skip_by_ukf) {
        to_print << "[Tracker][compute_tracking][seq:" << seq_id << "] skipping this frame\n";
        std::cout << to_print.str();
        return;
    }

    Eigen::Isometry3d pose_isom;
    pose_isom.matrix() = pose.cast<double>();

    if(this->is_first_frame) {
        this->is_first_frame = false;

        Keyframe::Ptr new_keyframe = create_keyframe();
        notify_keyframe_observers(new_keyframe);

        Header header;
        header.sequence = cloud->header.seq;
        header.timestamp = cloud->header.stamp;
        header.frame_id = cloud->header.frame_id;
        notify_odom_observers(header, pose);
    } else {
        if(update_odometry(pose, cloud->header.stamp, seq_id)) {
            Keyframe::Ptr new_keyframe = create_keyframe();
            notify_keyframe_observers(new_keyframe);

            Header header;
            header.sequence = cloud->header.seq;
            header.timestamp = cloud->header.stamp;
            header.frame_id = cloud->header.frame_id;
            notify_odom_observers(header, pose);
        }
    }

    this->odoms.emplace_back(pose);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    this->times.emplace_back(duration.count());
    this->total_time += duration.count();
    to_print << "[Tracker] TEMPO TEMPO: " << duration.count() << std::endl;
    to_print << "[Tracker][compute_tracking][seq:" << seq_id << "] finished tracking\n";
    std::cout << to_print.str();
}

/* ----------------------------------------------- */
/* Finds the relative motion between point clouds. */
/* ----------------------------------------------- */
Eigen::Matrix4f Tracker::matching(const uint64_t timestamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    int seq_id = cloud->header.seq;
    std::ostringstream to_print;

    to_print << "[Tracker][matching][seq:" << seq_id << "] matching point clouds\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    if (!this->keyframe_cloud) {
        to_print << "[Tracker][matching][seq:" << seq_id << "] setting the first keyframe\n";
        std::cout << to_print.str();
        to_print.str("");
        to_print.clear();

        this->prev_trans.setIdentity();
        this->keyframe_pose.setIdentity();
        this->keyframe_stamp = timestamp;
        this->keyframe_cloud = downsample(cloud);
        this->curr_filtered_cloud = this->keyframe_cloud;
        this->registration_method->setInputTarget(this->keyframe_cloud);
        this->keyframe_prior_odom.setIdentity();
        return Eigen::Matrix4f::Identity();
    }

    Eigen::Matrix4f motion_guess = compute_prior_odom_guess(timestamp);

    // if the ukf tells to skip the frame, return a dummy value
    if(this->skip_by_ukf) {
        return Eigen::Matrix4f::Identity();
    }

    pcl::PointCloud<PointT>::ConstPtr filtered_cloud = downsample(cloud);
    this->curr_filtered_cloud = filtered_cloud;
    this->registration_method->setInputSource(filtered_cloud);

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>);
    this->registration_method->align(*aligned, motion_guess);

    // scan matching not convergent
    if(!this->registration_method->hasConverged()) {
        to_print << "[Tracker][matching][seq:" << seq_id << "] scan matching has not converged, ignoring this frame\n";
        std::cout << to_print.str();
        return (this->keyframe_pose * this->prev_trans); // assumes constant motion
    }

    Eigen::Matrix4f trans = this->registration_method->getFinalTransformation(); // rigid motion between frames
    this->current_fitness = this->registration_method->getFitnessScore();
    Eigen::Matrix4f odom = this->keyframe_pose*trans; // the new pose

    if(this->transform_thresholding) {
        Eigen::Matrix4f delta = prev_trans.inverse() * trans;
        double dx = delta.block<3, 1>(0, 3).norm();
        double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());

        if(dx > this->max_acceptable_trans || da > this->max_acceptable_angle) {
            to_print << "[Tracker][matching][seq:" << seq_id << "] too large transform (" << dx << "[m], " << da << "[rad]), ignoring this frame\n";
            std::cout << to_print.str();
            return (this->keyframe_pose * this->prev_trans); // assumes constant motion
        }
    }

    this->prev_trans = trans;

    to_print << "[Tracker][matching][seq:" << seq_id << "] finished matching point clouds\n";
    std::cout << to_print.str();

    return odom;
}

/* -------------------------- */
/* Downsamples a point cloud. */
/* -------------------------- */
pcl::PointCloud<PointT>::ConstPtr Tracker::downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    int seq_id = cloud->header.seq;
    std::ostringstream to_print;
    to_print << "[Tracker][downsample][seq:" << seq_id << "] downsampling\n";

    if(!this->downsample_filter) {
        to_print << "[Tracker][downsample][seq:" << seq_id << "] no downsampling method assigned, skipping\n";
        std::cout << to_print.str();
        return cloud;
    }

    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
    this->downsample_filter->setInputCloud(cloud);
    this->downsample_filter->filter(*filtered_cloud);

    to_print << "[Prefilterer][downsample][seq:" << seq_id << "] finished downsampling\n";
    std::cout << to_print.str();

    return filtered_cloud;
}

/* ----------------------------------------------------------------------------------------- */
/* Updates the current odometry, Updates the current odometry, searching for a new keyframe. */
/* ----------------------------------------------------------------------------------------- */
bool Tracker::update_odometry(const Eigen::Matrix4f& odom, const uint64_t timestamp, int seq_id) {
    std::ostringstream to_print;

    to_print << "[Tracker][update_odometry][seq:" << seq_id << "] updating the tracker odometry settings\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    double delta_trans = this->prev_trans.block<3, 1>(0, 3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(this->prev_trans.block<3, 3>(0, 0)).w());
    uint64_t delta_time = timestamp - this->keyframe_stamp;
    if(delta_trans > this->keyframe_delta_trans || delta_angle > this->keyframe_delta_angle || delta_time > this->keyframe_delta_time*1e9) {
        this->keyframe_cloud = this->curr_filtered_cloud;
        this->registration_method->setInputTarget(this->keyframe_cloud);
        this->keyframe_pose = odom;
        this->keyframe_stamp = timestamp;
        this->prev_trans.setIdentity();
        this->accumulated_distance += delta_trans;
        this->keyframe_prior_odom = this->current_prior_odom;

        to_print << "[Tracker][update_odometry][seq:" << seq_id << "] new keyframe selected\n";
        to_print << "[Tracker][update_odometry][seq:" << seq_id << "] finished updating the tracker odometry settings\n";
        std::cout << to_print.str();
        return true;
    }

    to_print << "[Tracker][update_odometry][seq:" << seq_id << "] finished updating the tracker odometry settings\n";
    std::cout << to_print.str();

    return false;
}

/* ----------------------- */
/* Creates a new keyframe. */
/* ----------------------- */
Keyframe::Ptr Tracker::create_keyframe() {
    Eigen::Isometry3d odom_isom;
    odom_isom.matrix() = this->keyframe_pose.cast<double>();
    std::cout << odom_isom.matrix() << std::endl;
    Keyframe::Ptr new_keyframe(new Keyframe(this->keyframe_stamp, odom_isom, 0, this->keyframe_cloud));
    new_keyframe->accum_distance = this->accumulated_distance;
    new_keyframe->fitness_score = this->current_fitness;
    return new_keyframe;
}

/* ------------------------------------------------------------------------------------------------ */
/* Computes the motion guess to use when scan matching using the available pre-computed odometries. */
/* ------------------------------------------------------------------------------------------------ */
Eigen::Matrix4f Tracker::compute_prior_odom_guess(const uint64_t timestamp) {
    std::lock_guard<std::mutex> lock(this->prior_odom_mutex);

    Eigen::Matrix4f closest_prior_odom = find_closest_prior_odom(timestamp);
    this->current_prior_odom = closest_prior_odom;

    // if there is no guess, use the previous transformation as guess
    if(!this->has_current_prior_odom) {
        this->skip_by_ukf = false;
        return this->prev_trans;
    } else {
        // if there is no prior corresponding to the previous keyframe, use the previous transformation as a guess
        /*if(this->keyframe_prior_odom.isIdentity(1e-4)) {
            this->skip_by_ukf = false;
            return this->prev_trans;
        }*/
        Eigen::Matrix4f prior_trans = this->keyframe_prior_odom.inverse() * closest_prior_odom;

        double delta_trans = prior_trans.block<3, 1>(0, 3).norm();
        double delta_angle = std::acos(Eigen::Quaternionf(prior_trans.block<3, 3>(0, 0)).w());
        if (delta_trans > this->prior_odom_delta_trans || delta_angle > this->prior_odom_delta_angle) {
            this->skip_by_ukf = false;
        } else {
            this->skip_by_ukf = true;
        }

        return prior_trans;
    }
}

/* ------------------------------------------------------------------------------------ */
/* Finds the pre-computed odometry closest in time to the current frame (not keyframe). */
/* ------------------------------------------------------------------------------------ */
Eigen::Matrix4f Tracker::find_closest_prior_odom(const uint64_t timestamp) {
    // if there are no new pre-computed odometries, skip
    if(this->prior_odoms.empty()) {
        this->has_current_prior_odom = false;
        return Eigen::Matrix4f::Identity();
    }

    // find the pre-computed odometry that is closest to the current frame
    auto prior_odom_t_cursor = this->prior_odoms.begin();
    auto closest_prior_odom = prior_odom_t_cursor;
    for(auto prior_odom_t = prior_odom_t_cursor; prior_odom_t != this->prior_odoms.end(); prior_odom_t++) {
        uint64_t dt = ((*closest_prior_odom).first > timestamp) ? (*closest_prior_odom).first - timestamp : timestamp - (*closest_prior_odom).first;
        uint64_t dt2 = ((*prior_odom_t).first > timestamp) ? (*prior_odom_t).first - timestamp : timestamp - (*prior_odom_t).first;
        if(dt < dt2) {
            break;
        }

        closest_prior_odom = prior_odom_t;
    }

    // if there is a too large time gap between the current frame and the closest pre-computed odometry, skip
    uint64_t dt = ((*closest_prior_odom).first > timestamp) ? (*closest_prior_odom).first - timestamp : timestamp - (*closest_prior_odom).first;
    if(dt > 0.2*1e9) {
        this->has_current_prior_odom = false;
        return Eigen::Matrix4f::Identity();
    }

    // delete all the previous pre-computed odometries
    auto remove_loc = std::upper_bound(this->prior_odoms.begin(), this->prior_odoms.end(), timestamp,
                                       [=](uint64_t timestamp, const std::pair<uint64_t, Eigen::Matrix4f>& prior_odom){return prior_odom.first > timestamp;});
    this->prior_odoms.erase(this->prior_odoms.begin(), remove_loc);

    this->has_current_prior_odom = true;
    return (*closest_prior_odom).second;
}

/* ---------------------------------------------------------------------------- */
/* Tells the tracker that a point cloud is ready to be dispatched for tracking. */
/* ---------------------------------------------------------------------------- */
void Tracker::dispatch_tracking(const pcl::PointCloud<PointT>::ConstPtr &cloud) {
    this->tracker_dqueue->dispatch([this, cloud]{this->compute_tracking(cloud);});
}

/* ------------------------------------------------------------------------------ */
/* Tells the tracker that prior odometry is ready to be dispatched for insertion. */
/* ------------------------------------------------------------------------------ */
void Tracker::dispatch_prior_odom_insertion(const Header& header, const Eigen::Matrix4f& odom) {
    this->prior_odom_insertion_dqueue->dispatch([this, header, odom]{this->add_prior_odom(header, odom);});
}

/* ---------------------------------------------------------------------------------------- */
/* Adds a pre-computed odometry, to be used when performing scan matching as initial guess. */
/* ---------------------------------------------------------------------------------------- */
void Tracker::add_prior_odom(const Header& header, const Eigen::Matrix4f& odom) {
    std::lock_guard<std::mutex> lock(this->prior_odom_mutex);
    this->prior_odoms.emplace_back(std::make_pair(header.timestamp, odom));
}

/* ------------------------------------------------------- */
/* Saves the prefiltering times to file, for benchmarking. */
/* ------------------------------------------------------- */
void Tracker::dump_times(const std::string& filename) {
    std::ofstream output_file(filename);

    std::ostream_iterator<uint64_t> output_iterator(output_file, "\n");
    std::copy(this->times.begin(), this->times.end(), output_iterator);
}