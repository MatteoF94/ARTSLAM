#include "ground_detector.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>

using namespace artslam::laser3d;

GroundDetector::GroundDetector() {
    std::stringstream msg;

    if(configuration_.verbose_) {
        msg << "[GroundDetector] Creating and configuring the ground detector\n";
        std::cout << msg.str();
        msg.str("");
    }

    ground_detector_dispatcher_ = std::make_unique<Dispatcher>("GroundDetectorDispatcher", 1);

    if(configuration_.verbose_) {
        msg << "[GroundDetector] Finished creating and configuring the tracker\n";
        std::cout << msg.str();
    }
}

GroundDetector::GroundDetector(const Configuration &configuration) {
    std::stringstream msg;
    configuration_.verbose_ = configuration.verbose_;

    if(configuration_.verbose_) {
        msg << "[GroundDetector] Creating and configuring the ground detector\n";
        std::cout << msg.str();
        msg.str("");
    }

    configuration_.tilt_angle_ = configuration.tilt_angle_;
    configuration_.sensor_height_ = configuration.sensor_height_;
    configuration_.clipping_range_ = configuration.clipping_range_;
    configuration_.floor_points_threshold_ = configuration.floor_points_threshold_;
    configuration_.use_normal_filtering_ = configuration.use_normal_filtering_;
    configuration_.normal_filtering_threshold_ = configuration.normal_filtering_threshold_;
    configuration_.rough_ground_check_ = configuration.rough_ground_check_;

    ground_detector_dispatcher_ = std::make_unique<Dispatcher>("GroundDetectorDispatcher", 1);

    if(configuration_.verbose_) {
        msg << "[GroundDetector] Finished creating and configuring the ground detector\n";
        std::cout << msg.str();
    }
}

// Signals that a new filtered point cloud has been received
void GroundDetector::update_filtered_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) {
    ground_detector_dispatcher_->dispatch([this, pointcloud]{detect_ground_model(pointcloud);});
}

// Registers an object waiting for floor coefficients
void GroundDetector::register_floor_coefficients_observer(FloorCoefficientsObserver *floor_coefficients_observer) {
    floor_coefficients_observers_.emplace_back(floor_coefficients_observer);

}

// Removes an object waiting for floor coefficients
void GroundDetector::remove_floor_coefficients_observer(FloorCoefficientsObserver *floor_coefficients_observer) {
    auto iterator = std::find(floor_coefficients_observers_.begin(), floor_coefficients_observers_.end(), floor_coefficients_observer);

    if(iterator != floor_coefficients_observers_.end()) {
        floor_coefficients_observers_.erase(iterator);
    }
}

// Notifies an object waiting for floor coefficients
void GroundDetector::notify_floor_coefficients_observers(const FloorCoefficients_MSG::ConstPtr& floor_coefficients_msg) {
    for(FloorCoefficientsObserver* observer : floor_coefficients_observers_) {
        observer->update_floor_coefficients_observer(floor_coefficients_msg);
    }
}

void GroundDetector::detect_ground_model(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud) {
    auto start = std::chrono::high_resolution_clock::now();
    std::stringstream msg;

    if(configuration_.verbose_) {
        msg << "[GroundDetector] Detecting ground model of cloud with (id, timestamp): (" << pointcloud->header.seq << ", " << pointcloud->header.stamp
            << ")\n";
        std::cout << msg.str();
        msg.str("");
    }

    pcl::PointCloud<Point3I>::Ptr filtered_pointcloud(new pcl::PointCloud<Point3I>());

    filtered_pointcloud = filter_pointcloud(pointcloud);

    std::optional<EigVector4d> floor_coefficients = detect_floor_coefficients(filtered_pointcloud);

    if(floor_coefficients.has_value()) {
        FloorCoefficients_MSG::Ptr floor_coefficients_msg = std::make_shared<FloorCoefficients_MSG>();
        floor_coefficients_msg->header_.timestamp_ = pointcloud->header.stamp;
        floor_coefficients_msg->header_.sequence_ = pointcloud->header.seq;
        floor_coefficients_msg->header_.frame_id_ = pointcloud->header.frame_id;
        floor_coefficients_msg->coefficients_ = floor_coefficients.value();

        notify_floor_coefficients_observers(floor_coefficients_msg);

        if(configuration_.verbose_) {
            msg << "[GroundDetector] Detected coefficients: \n\t" << floor_coefficients.value() << "\n";
            std::cout << msg.str();
            msg.str("");
        }
    } else if(configuration_.rough_ground_check_) {
        std::optional<EigVector4d> ground_coefficients = detect_rough_ground_coefficients(filtered_pointcloud);

        if(ground_coefficients.has_value()) {
            FloorCoefficients_MSG::Ptr ground_coefficients_msg = std::make_shared<FloorCoefficients_MSG>();
            ground_coefficients_msg->header_.timestamp_ = pointcloud->header.stamp;
            ground_coefficients_msg->header_.sequence_ = pointcloud->header.seq;
            ground_coefficients_msg->header_.frame_id_ = pointcloud->header.frame_id;
            ground_coefficients_msg->coefficients_ = floor_coefficients.value();

            notify_floor_coefficients_observers(ground_coefficients_msg);
        }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    total_time_ += duration.count();
    count_++;

    if(configuration_.verbose_) {
        msg << "[GroundDetector] Total time and count: " << duration.count() << ", " << count_ << "\n";
        std::cout << msg.str();
    }
}

pcl::PointCloud<Point3I>::Ptr GroundDetector::filter_pointcloud(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) {
    pcl::PointCloud<Point3I>::Ptr filtered_pointcloud(new pcl::PointCloud<Point3I>());

    EigMatrix4f tilt_matrix = EigMatrix4f::Identity();
    tilt_matrix.topLeftCorner(3,3) = EigAngleAxisf(configuration_.tilt_angle_ * M_PI / 180.0f, EigVector3f::UnitY()).toRotationMatrix();
    pcl::transformPointCloud(*pointcloud, *filtered_pointcloud, tilt_matrix);

    filtered_pointcloud = clip_with_plane(filtered_pointcloud, EigVector4f(0.0f, 0.0f, 1.0f, configuration_.sensor_height_ + configuration_.clipping_range_), false);
    filtered_pointcloud = clip_with_plane(filtered_pointcloud, EigVector4f(0.0f, 0.0f, 1.0f, configuration_.sensor_height_ - configuration_.clipping_range_), true);

    if(configuration_.use_normal_filtering_) {
        filtered_pointcloud = filter_normals(filtered_pointcloud);
    }

    pcl::transformPointCloud(*filtered_pointcloud, *filtered_pointcloud, tilt_matrix.matrix().inverse());
    return filtered_pointcloud;
}

pcl::PointCloud<Point3I>::Ptr GroundDetector::clip_with_plane(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud,
                                                              const EigVector4f& plane,
                                                              bool negative) const {
    pcl::PlaneClipper3D<Point3I> plane_clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices());
    plane_clipper.clipPointCloud3D(*pointcloud, indices->indices);

    pcl::PointCloud<Point3I>::Ptr clipped_pointcloud(new pcl::PointCloud<Point3I>());
    pcl::ExtractIndices<Point3I> extract_indices;
    extract_indices.setInputCloud(pointcloud);
    extract_indices.setIndices(indices);
    extract_indices.setNegative(negative);
    extract_indices.filter(*clipped_pointcloud);

    return clipped_pointcloud;
}

pcl::PointCloud<Point3I>::Ptr GroundDetector::filter_normals(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud) const {
    pcl::NormalEstimation<Point3I, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(pointcloud);

    pcl::search::KdTree<Point3I>::Ptr kdtree(new pcl::search::KdTree<Point3I>());
    normal_estimation.setSearchMethod(kdtree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    normal_estimation.setKSearch(10);
    normal_estimation.setViewPoint(0.0f,0.0f, configuration_.sensor_height_);
    normal_estimation.compute(*normals);

    pcl::PointCloud<Point3I>::Ptr filtered_pointcloud(new pcl::PointCloud<Point3I>());
    filtered_pointcloud->reserve(pointcloud->size());

    for(int i = 0; i < pointcloud->size(); i++) {
        float dot = normals->at(i).getNormalVector3fMap().normalized().dot(EigVector3f::UnitZ());
        if(std::abs(dot) > std::cos(configuration_.normal_filtering_threshold_ * M_PI / 180.0)) {
            filtered_pointcloud->push_back(pointcloud->at(i));
        }
    }

    filtered_pointcloud->width = filtered_pointcloud->size();
    filtered_pointcloud->height = 1;
    filtered_pointcloud->is_dense = false;
    return filtered_pointcloud;
}

std::optional<EigVector4d>
GroundDetector::detect_floor_coefficients(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud) const {
    if(pointcloud->size() < configuration_.floor_points_threshold_) {
        return {};
    }

    pcl::SampleConsensusModelPlane<Point3I>::Ptr plane_model(new pcl::SampleConsensusModelPlane<Point3I>(pointcloud));
    pcl::RandomSampleConsensus<Point3I> ransac(plane_model);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers_indices(new pcl::PointIndices());
    ransac.getInliers(inliers_indices->indices);

    if(inliers_indices->indices.size() < configuration_.floor_points_threshold_) {
        return {};
    }

    EigMatrix4f tilt_matrix = EigMatrix4f::Identity();
    tilt_matrix.topLeftCorner(3,3) = EigAngleAxisf(configuration_.tilt_angle_ * M_PI / 180.0, EigVector3f::UnitY()).toRotationMatrix();
    EigVector4f reference = tilt_matrix.inverse() * EigVector4f::UnitZ();
    EigVectorXf coefficients;
    ransac.getModelCoefficients(coefficients);
    double dot = coefficients.head<3>().dot(reference.head<3>());
    if(std::abs(dot) < std::cos(configuration_.normal_filtering_threshold_ * M_PI / 180.0)) {
        return {};
    }

    if(coefficients.head<3>().dot(EigVector3f::UnitZ()) < 0.0f) {
        coefficients *= -1;
    }

    return EigVector4d(coefficients.cast<double>());
}

std::optional<EigVector4d>
GroundDetector::detect_rough_ground_coefficients(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud) const {
    pcl::PointCloud<Point3I>::Ptr filtered_pointcloud(new pcl::PointCloud<Point3I>());
    if(pointcloud->size() < configuration_.floor_points_threshold_) {
        return {};
    }

    // take only the points within the threshold boundaries
    std::copy_if(pointcloud->begin(), pointcloud->end(), std::back_inserter(filtered_pointcloud->points), [&](const Point3I& p) {
        double d = p.getVector3fMap().norm();
        return d < configuration_.rough_ground_max_dist_;
    });

    int count = filtered_pointcloud->size();
    double mean_x = 0, mean_y = 0, mean_z = 0;
    double mean_xx = 0, mean_yy = 0, mean_zz = 0;
    double mean_xy = 0, mean_xz = 0, mean_yz = 0;
    for(auto p : *filtered_pointcloud) {
        mean_x += p.x;
        mean_y += p.y;
        mean_z += p.z;

        mean_xx += p.x * p.x;
        mean_yy += p.y * p.y;
        mean_zz += p.z * p.z;

        mean_xy += p.x * p.y;
        mean_xz += p.x * p.z;
        mean_yz += p.y * p.z;
    }

    mean_x /= count;
    mean_y /= count;
    mean_z /= count;
    mean_xx /= count;
    mean_yy /= count;
    mean_zz /= count;
    mean_xy /= count;
    mean_xz /= count;
    mean_yz /= count;

    EigMatrix3d emat;
    emat(0,0) = mean_xx - mean_x * mean_x;
    emat(0,1) = mean_xy - mean_x * mean_y;
    emat(0,2) = mean_xz - mean_x * mean_z;
    emat(1,0) = mean_xy - mean_x * mean_y;
    emat(1,1) = mean_yy - mean_y * mean_y;
    emat(1,2) = mean_yz - mean_y * mean_z;
    emat(2,0) = mean_xz - mean_x * mean_z;
    emat(2,1) = mean_yz - mean_y * mean_z;
    emat(2,2) = mean_zz - mean_z * mean_z;
    Eigen::EigenSolver<EigMatrix3d> solver(emat);
    EigMatrix3d evalue = solver.pseudoEigenvalueMatrix();
    EigMatrix3d evector = solver.pseudoEigenvectors();

    double v1 = evalue(0,0);
    double v2 = evalue(1,1);
    double v3 = evalue(2,2);
    int min_number = 0;
    if((std::abs(v2) <= std::abs(v1)) && (std::abs(v2) <= std::abs(v3))) {
        min_number = 1;
    }
    if((std::abs(v3) <= std::abs(v1)) && (std::abs(v3) <= std::abs(v2))) {
        min_number = 2;
    }
    double a = evector(0, min_number);
    double b = evector(1, min_number);
    double c = evector(2, min_number);
    double d = -(a * mean_x + b * mean_y + c * mean_z);

    if(c < 0) {
        a *= -1;
        b *= -1;
        c *= -1;
        d *= -1;
    }

    EigMatrix4d tilt_matrix = EigMatrix4d::Identity();
    tilt_matrix.topLeftCorner(3,3) = EigAngleAxisd(configuration_.tilt_angle_ * M_PI / 180.0, EigVector3d::UnitY()).toRotationMatrix();
    EigVector4d reference = tilt_matrix.inverse() * EigVector4d::UnitZ();
    EigVector4d coefficients(a, b, c, d);
    coefficients /= coefficients.head<3>().norm();
    double dot = coefficients.head<3>().dot(reference.head<3>());
    if(std::abs(dot) < std::cos(configuration_.normal_filtering_threshold_ * M_PI / 180.0)) {
        return {};
    }

    return coefficients;
}