/** @file floor_detector.cpp
 * @brief Definition of class FloorDetector
 * @author Matteo Frosi
 */

#include <floor_detector.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>

FloorDetector::FloorDetector() {
    FloorDetectorConfig default_config;
    configure_floor_detector(&default_config);
}

FloorDetector::FloorDetector(const FloorDetectorConfig *const config_ptr) {
    configure_floor_detector(config_ptr);
}

FloorDetector::~FloorDetector() {
    std::ostringstream to_print;
    to_print << "[FloorDetector] destroying the floor detector\n";
    std::cout << to_print.str();
    delete this->floor_detector_queue;
}

/**
 * @brief Configures the floor detector object using a configuration object.
 */
void FloorDetector::configure_floor_detector(const FloorDetectorConfig *const config_ptr) {
    std::ostringstream to_print;

    to_print << "[FloorDetector][configure_floor_detector] creating and configurating the floor detector\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    this->tilt_deg = config_ptr->tilt_deg;
    this->sensor_height = config_ptr->sensor_height;
    this->height_clip_range = config_ptr->height_clip_range;
    this->floor_pts_thresh = config_ptr->floor_pts_thresh;
    this->floor_normal_thresh = config_ptr->floor_normal_thresh;
    this->use_normal_filtering = config_ptr->use_normal_filtering;
    this->normal_filter_thresh = config_ptr->normal_filter_thresh;

    this->floor_detector_queue = new DispatchQueue("floor detection queue",1);

    to_print << "[FloorDetector][configure_floor_detector] TILT_DEG: " << this->tilt_deg << "\n";
    to_print << "[FloorDetector][configure_floor_detector] SENSOR_HEIGHT: " << this->sensor_height << "\n";
    to_print << "[FloorDetector][configure_floor_detector] HEIGHT_CLIP_RANGE: " << this->height_clip_range << "\n";
    to_print << "[FloorDetector][configure_floor_detector] FLOOR_PTS_THRESH: " << this->floor_pts_thresh << "\n";
    to_print << "[FloorDetector][configure_floor_detector] FLOOR_NORMAL_THRESH: " << this->floor_normal_thresh << "\n";
    to_print << "[FloorDetector][configure_floor_detector] USE_NORMAL_FILTERING: " << this->use_normal_filtering << "\n";
    to_print << "[FloorDetector][configure_floor_detector] NORMAL_FILTER_THRESH: " << this->normal_filter_thresh << "\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    to_print << "[FloorDetector][configure_floor_detector] finished creating and configurating the floor detector\n";
    std::cout << to_print.str();
}

/**
 * @brief Registers an object waiting for floor coefficients.
 */
void FloorDetector::register_floor_coeffs_observer(FloorCoeffsObserver *fc_observer_ptr_tmp) {
    this->floor_coeffs_observers.emplace_back(fc_observer_ptr_tmp);
}

/**
 * @brief Removes an object waiting for floor coefficients.
 */
void FloorDetector::remove_floor_coeffs_observer(FloorCoeffsObserver *fc_observer_ptr_tmp) {
    auto iterator = std::find(this->floor_coeffs_observers.begin(), this->floor_coeffs_observers.end(), fc_observer_ptr_tmp);

    if(iterator != this->floor_coeffs_observers.end()) {
        this->floor_coeffs_observers.erase(iterator);
    }
}

/**
 * @brief Notifies all the observers with estimated floor coefficients.
 */
void FloorDetector::notify_floor_coeffs_observers(const FloorCoeffsMSG::ConstPtr& floor_coeffs_msg) {
    for(FloorCoeffsObserver *observer : this->floor_coeffs_observers) {
        observer->update(floor_coeffs_msg);
    }
}

/**
 * @brief It signals that a new point cloud has been received.
 */
void FloorDetector::update(pcl::PointCloud<PointT>::ConstPtr src_cloud) {
    dispatch_floor_detection(src_cloud);
}

/**
 * @brief It detects a floor in a point cloud.
 */
void FloorDetector::floor_detection(const pcl::PointCloud<PointT>::ConstPtr &cloud) {
    int seq_id = cloud->header.seq;
    std::ostringstream to_print;

    to_print << "[FloorDetector][floor_detection][seq:" << seq_id << "] searching for a floor in the point cloud\n";

    if(cloud->empty()) {
        to_print << "[FloorDetector][floor_detection][seq:" << seq_id << "] empty cloud, skipping\n";
        std::cout << to_print.str();
        return;
    }

    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    boost::optional<Eigen::Vector4d> floor = detect(cloud);

    if(floor) {
        FloorCoeffsMSG::Ptr floor_coeffs_msg = std::make_shared<FloorCoeffsMSG>();
        floor_coeffs_msg->header.frame_id = cloud->header.frame_id;
        floor_coeffs_msg->header.timestamp = cloud->header.stamp;
        floor_coeffs_msg->header.sequence = cloud->header.seq;
        floor_coeffs_msg->floor_coeffs = floor.get();

        to_print << "[FloorDetector][floor_detection][seq:" << seq_id << "] header.frame_id " << floor_coeffs_msg->header.frame_id << "\n";
        to_print << "[FloorDetector][floor_detection][seq:" << seq_id << "] header.timestamp " << floor_coeffs_msg->header.timestamp << "\n";
        to_print << "[FloorDetector][floor_detection][seq:" << seq_id << "] header.sequence " << floor_coeffs_msg->header.sequence << "\n";
        to_print << "[FloorDetector][floor_detection][seq:" << seq_id << "] floor_coeffs " << floor_coeffs_msg->floor_coeffs << "\n";

        // notifies all objects waiting for a filtered point cloud
        notify_floor_coeffs_observers(floor_coeffs_msg);
    }

    to_print << "[FloorDetector][floor_detection][seq:" << seq_id << "] finished searching for a floor\n";
    std::cout << to_print.str();
}

/**
 * @brief Performs the main operations needed to detect a floor.
 */
boost::optional<Eigen::Vector4d> FloorDetector::detect(const pcl::PointCloud<PointT>::ConstPtr &cloud) const {
    int seq_id = cloud->header.seq;
    std::ostringstream to_print;

    to_print << "[FloorDetector][detect][seq:" << seq_id << "] detecting a floor\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    // compensate for the tilt of the sensor
    Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
    tilt_matrix.topLeftCorner(3,3) = Eigen::AngleAxisf(this->tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();

    // filtering (clipping + normal)
    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*cloud, *filtered_cloud, tilt_matrix); // transform to have floor parallel to ground plane
    filtered_cloud = plane_clip(filtered_cloud, Eigen::Vector4f(0.0f,0.0f,1.0f,this->sensor_height + this->height_clip_range), false);
    filtered_cloud = plane_clip(filtered_cloud, Eigen::Vector4f(0.0f,0.0f,1.0f,this->sensor_height - this->height_clip_range), true);

    if(this->use_normal_filtering) {
        filtered_cloud = normal_filtering(filtered_cloud);
    }

    pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, static_cast<Eigen::Matrix4f>(tilt_matrix.inverse())); // transform back

    // RANSAC
    // cloud is too small
    if(filtered_cloud->size() < this->floor_pts_thresh) {
        to_print << "[FloorDetector][detect][seq:" << seq_id << "] too few points, skipping\n";
        std::cout << to_print.str();
        return boost::none;
    }

    pcl::SampleConsensusModelPlane<PointT>::Ptr model_plane(new pcl::SampleConsensusModelPlane<PointT>(filtered_cloud));
    pcl::RandomSampleConsensus<PointT> ransac(model_plane);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    ransac.getInliers(inliers->indices);

    // too few inliers
    if(inliers->indices.size() < this->floor_pts_thresh) {
        to_print << "[FloorDetector][detect][seq:" << seq_id << "] too few inliers from RANSAC, skipping\n";
        std::cout << to_print.str();
        return boost::none;
    }

    // check the normal of the detected floor
    Eigen::Vector4f reference = tilt_matrix.inverse() * Eigen::Vector4f::UnitZ();
    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);
    double dot = coeffs.head<3>().dot(reference.head<3>());
    if(std::abs(dot) < std::cos(this->floor_normal_thresh * M_PI / 180.f)) {
        to_print << "[FloorDetector][detect][seq:" << seq_id << "] the normal of the detected floor is not vertical, skipping\n";
        std::cout << to_print.str();
        //TODO this can be reworked to find also oblique planes, e.g. ramps
        return boost::none;
    }

    // make the normal upward if it is not (may happen)
    if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
        coeffs *= -1.0f;
    }

    to_print << "[FloorDetector][detect][seq:" << seq_id << "] finished detecting a floor\n";
    std::cout << to_print.str();

    return Eigen::Vector4d(coeffs.cast<double>());
}

/**
 * @brief Clips the point cloud to reduce the range search of the floor.
 */
pcl::PointCloud<PointT>::Ptr FloorDetector::plane_clip(const pcl::PointCloud<PointT>::Ptr &src_cloud,
                                                       const Eigen::Vector4f &plane, bool negative) const {
    int seq_id = src_cloud->header.seq;
    std::ostringstream to_print;

    to_print << "[FloorDetector][plane_clip][seq:" << seq_id << "] clipping the point cloud\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    pcl::PlaneClipper3D<PointT> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    clipper.clipPointCloud3D(*src_cloud, indices->indices);

    pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract_indices;
    extract_indices.setInputCloud(src_cloud);
    extract_indices.setIndices(indices);
    extract_indices.setNegative(negative);
    extract_indices.filter(*dst_cloud);

    to_print << "[FloorDetector][plane_clip][seq:" << seq_id << "] finished clipping the point cloud\n";
    std::cout << to_print.str();

    return dst_cloud;
}

/**
 * @brief Filters the point cloud depending on the normals of its points, after having computed them.
 */
pcl::PointCloud<PointT>::Ptr FloorDetector::normal_filtering(const pcl::PointCloud<PointT>::Ptr &cloud) const {
    int seq_id = cloud->header.seq;
    std::ostringstream to_print;

    to_print << "[FloorDetector][normal_filtering][seq:" << seq_id << "] filtering the point cloud using the normals of its points\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    pcl::NormalEstimation<PointT,pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);

    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
    normal_estimation.setSearchMethod(kdtree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    normal_estimation.setKSearch(10);
    normal_estimation.setViewPoint(0.0f,0.0f,this->sensor_height);
    normal_estimation.compute(*normals);

    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
    filtered_cloud->reserve(cloud->size());

    for(int i = 0; i < cloud->size(); i++) {
        float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
        if(std::abs(dot) > std::cos(this->normal_filter_thresh * M_PI / 180.f)) {
            filtered_cloud->push_back(cloud->at(i));
        }
    }

    filtered_cloud->width = filtered_cloud->size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;

    to_print << "[FloorDetector][normal_filtering][seq:" << seq_id << "] finished filtering the point cloud\n";
    std::cout << to_print.str();

    return filtered_cloud;
}

/**
 * @brief Tells the floor detector that a point cloud is ready to be dispatched for floor detection.
 */
void FloorDetector::dispatch_floor_detection(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    this->floor_detector_queue->dispatch([this, cloud]{this->floor_detection(cloud);});
}
