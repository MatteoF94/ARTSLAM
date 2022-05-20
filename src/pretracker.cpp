#include "pretracker.h"
#include <utility>
#include <chrono>

using namespace artslam::laser3d;

// Class constructor
Pretracker::Pretracker(pcl::Registration<Point3I, Point3I>::Ptr lsrm, pcl::Registration<Point3I, Point3I>::Ptr msrm) {
    std::stringstream msg;

    if(verbose_) {
        msg << "[Pretracker] Creating and configuring the pretracker\n";
        std::cout << msg.str();
        msg.str("");
    }

    low_scale_pointcloud_ = pcl::PointCloud<Point3I>::Ptr(new pcl::PointCloud<Point3I>());
    medium_scale_pointcloud_ = pcl::PointCloud<Point3I>::Ptr(new pcl::PointCloud<Point3I>());

    low_scale_registration_method_ = std::move(lsrm);
    medium_scale_registration_method_ = std::move(msrm);

    pretracker_dispatcher_ = std::make_unique<Dispatcher>("PretrackerDispatcher", 1);

    if(verbose_) {
        msg << "[Pretracker] Finished creating and configuring the pretracker\n";
        std::cout << msg.str();
    }
}

Pretracker::Pretracker(const Configuration &configuration, pcl::Registration<Point3I, Point3I>::Ptr lsrm,
                       pcl::Registration<Point3I, Point3I>::Ptr msrm) {
    std::stringstream msg;

    verbose_ = configuration.verbose_;

    if(verbose_) {
        msg << "[Pretracker] Creating and configuring the pretracker\n";
        std::cout << msg.str();
        msg.str("");
    }

    medium_scale_enabled_ = configuration.medium_scale_enabled_;
    low_scale_leaf_size_ = configuration.low_scale_leaf_size_;
    medium_scale_leaf_size_ = configuration.medium_scale_leaf_size_;

    low_scale_registration_method_ = std::move(lsrm);
    medium_scale_registration_method_ = std::move(msrm);

    pretracker_dispatcher_ = std::make_unique<Dispatcher>("PretrackerDispatcher", 1);

    if(verbose_) {
        msg << "[Pretracker] Finished creating and configuring the pretracker\n";
        std::cout << msg.str();
    }
}

// Signals that a new raw point cloud has been received
void Pretracker::update_raw_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) {
    pretracker_dispatcher_->dispatch([this, pointcloud]{pretrack(pointcloud);});
}

// Registers an object waiting for odometries
void Pretracker::register_odometry_observer(PriorOdometryObserver* odometry_observer) {
    odometry_observers_.emplace_back(odometry_observer);
}

// Removes an object waiting for odometries
void Pretracker::remove_odometry_observer(PriorOdometryObserver* odometry_observer) {
    auto iterator = std::find(odometry_observers_.begin(), odometry_observers_.end(), odometry_observer);

    if(iterator != odometry_observers_.end()) {
        odometry_observers_.erase(iterator);
    }
}

// Notifies all the objects waiting for filtered point clouds
void Pretracker::notify_odometry_observers(const OdometryStamped3D_MSG::ConstPtr& odometry3d_msg) {
    for(PriorOdometryObserver* observer : odometry_observers_) {
        observer->update_prior_odometry_observer(odometry3d_msg);
    }
}

void Pretracker::pretrack(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud) {
    if(pointcloud->empty()) {
        return;
    }

    if(first_frame_) {
        first_frame_ = false;
        handle_first_frame(pointcloud);
        return;
    }

    EigMatrix4f transformation = EigMatrix4f::Identity();
    transformation = pretrack_low_scale(pointcloud);

    if(medium_scale_enabled_) {
        transformation = pretrack_medium_scale(pointcloud, transformation);
    }

    odometry_ = odometry_ * transformation;
    //std::cout << odometry_ << std::endl;

    OdometryStamped3D_MSG::Ptr odometry_msg(new OdometryStamped3D_MSG());
    odometry_msg->header_.timestamp_ = pointcloud->header.stamp;
    odometry_msg->odometry_ = odometry_;

    notify_odometry_observers(odometry_msg);
}

void Pretracker::handle_first_frame(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) {
    low_scale_downsampler_.setLeafSize(low_scale_leaf_size_, low_scale_leaf_size_, low_scale_leaf_size_);
    low_scale_downsampler_.setInputCloud(pointcloud);
    low_scale_downsampler_.filter(*low_scale_pointcloud_);

    medium_scale_downsampler_.setLeafSize(medium_scale_leaf_size_, medium_scale_leaf_size_, medium_scale_leaf_size_);
    medium_scale_downsampler_.setInputCloud(pointcloud);
    medium_scale_downsampler_.filter(*medium_scale_pointcloud_);
}

EigMatrix4f Pretracker::pretrack_low_scale(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud) {
    std::stringstream msg;
    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<Point3I>::Ptr low_scale_pointcloud(new pcl::PointCloud<Point3I>());
    low_scale_downsampler_.setInputCloud(pointcloud);
    low_scale_downsampler_.filter(*low_scale_pointcloud);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if(verbose_) {
        msg << "[Pretracker] Low downsampling: " << duration.count() << "\n";
        std::cout << msg.str();
        msg.str("");
    }

    start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<Point3I>::Ptr aligned_pointcloud(new pcl::PointCloud<Point3I>());
    low_scale_registration_method_->setInputSource(low_scale_pointcloud);
    low_scale_registration_method_->setInputTarget(low_scale_pointcloud_);
    low_scale_registration_method_->align(*aligned_pointcloud);

    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if(verbose_) {
        msg << "[Pretracker] Low tracking: " << duration.count() << "\n";
        std::cout << msg.str();
    }

    low_scale_pointcloud_ = low_scale_pointcloud;

    if(!low_scale_registration_method_->hasConverged()) {
        return EigMatrix4f::Identity();
    }

    return low_scale_registration_method_->getFinalTransformation();
}

EigMatrix4f
Pretracker::pretrack_medium_scale(const pcl::PointCloud<Point3I>::ConstPtr &pointcloud, const EigMatrix4f &guess) {
    std::stringstream msg;
    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<Point3I>::Ptr medium_scale_pointcloud(new pcl::PointCloud<Point3I>());
    medium_scale_downsampler_.setInputCloud(pointcloud);
    medium_scale_downsampler_.filter(*medium_scale_pointcloud);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if(verbose_) {
        msg << "[Pretracker] Medium downsampling: " << duration.count() << "\n";
        std::cout << msg.str();
        msg.str("");
    }

    start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<Point3I>::Ptr aligned_pointcloud(new pcl::PointCloud<Point3I>());
    medium_scale_registration_method_->setInputSource(medium_scale_pointcloud);
    medium_scale_registration_method_->setInputTarget(medium_scale_pointcloud_);
    medium_scale_registration_method_->align(*aligned_pointcloud, guess);

    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if(verbose_) {
        msg << "[Pretracker] Medium tracking: " << duration.count() << "\n";
        std::cout << msg.str();
    }

    medium_scale_pointcloud_ = medium_scale_pointcloud;

    if(!medium_scale_registration_method_->hasConverged()) {
        return EigMatrix4f::Identity();
    }

    return medium_scale_registration_method_->getFinalTransformation();
}