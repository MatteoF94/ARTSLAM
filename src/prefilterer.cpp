#include "prefilterer.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <chrono>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

using namespace artslam::laser3d;

// Class constructor
Prefilterer::Prefilterer() {
    std::stringstream msg;

    if(configuration_.verbose_) {
        msg << "[Prefilterer] Creating and configuring the prefilterer\n";
        std::cout << msg.str();
        msg.str("");
    }

    configure_downsampler(configuration_.downsample_method_, configuration_.downsample_resolution_, configuration_.sample_size_);
    configure_outlier_remover(configuration_.outlier_removal_method_, configuration_.statistical_mean_k_, configuration_.statistical_stddev_, configuration_.radius_radius_, configuration_.radius_min_neighbours_);

    prefilterer_dispatcher_ = std::make_unique<Dispatcher>("PrefiltererDispatcher", 1);

    if(configuration_.verbose_) {
        msg << "[Prefilterer] Finished creating and configuring the prefilterer\n";
        std::cout << msg.str();
    }
}

// Class constructor, with parameters
Prefilterer::Prefilterer(const Configuration& configuration) {
    std::stringstream msg;
    configuration_.verbose_ = configuration.verbose_;

    if(configuration_.verbose_) {
        msg << "[Prefilterer] Creating and configurating the prefilterer\n";
        std::cout << msg.str();
    }

    configuration_.downsample_method_ = configuration.downsample_method_;
    configuration_.downsample_resolution_ = configuration.downsample_resolution_;
    configuration_.sample_size_ = configuration.sample_size_;
    configuration_.outlier_removal_method_ = configuration.outlier_removal_method_;
    configuration_.statistical_mean_k_ = configuration.statistical_mean_k_;
    configuration_.statistical_stddev_ = configuration.statistical_stddev_;
    configuration_.radius_radius_ = configuration.radius_radius_;
    configuration_.radius_min_neighbours_ = configuration.radius_min_neighbours_;
    configuration_.use_distance_filter_ = configuration.use_distance_filter_;
    configuration_.distance_near_threshold_ = configuration.distance_near_threshold_;
    configuration_.distance_far_threshold_ = configuration.distance_far_threshold_;
    configuration_.scan_period_ = configuration.scan_period_;
    for(int i = 0; i < 9; i++)
        configuration_.imu_to_lidar_rotation_[i] = configuration.imu_to_lidar_rotation_[i];

    configure_downsampler(configuration_.downsample_method_, configuration_.downsample_resolution_, configuration_.sample_size_);
    configure_outlier_remover(configuration_.outlier_removal_method_, configuration_.statistical_mean_k_, configuration_.statistical_stddev_, configuration_.radius_radius_, configuration_.radius_min_neighbours_);

    prefilterer_dispatcher_ = std::make_unique<Dispatcher>("PrefiltererDispatcher", 1);

    if(configuration_.verbose_) {
        msg.str("");
        msg << "[Prefilterer] Finished creating and configuring the prefilterer\n";
        std::cout << msg.str();
    }
}

// Class destructor
Prefilterer::~Prefilterer() = default;

// Configures the downsampling method used to filter point clouds
void Prefilterer::configure_downsampler(const std::string& method, float downsample_resolution, int sample_size) {
    std::stringstream msg;

    if(method == "VOXELGRID") {
        msg << "[Prefilterer] Downsampling with VOXELGRID method, resolution: " << downsample_resolution << "\n";
        pcl::VoxelGrid<Point3I>::Ptr voxelgrid(new pcl::VoxelGrid<Point3I>());
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        downsample_filter_ = voxelgrid;
    } else if(method == "APPROX_VOXELGRID") {
        msg << "[Prefilterer] Downsampling with APPROX_VOXELGRID method, resolution: " << downsample_resolution << "\n";
        pcl::ApproximateVoxelGrid<Point3I>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<Point3I>());
        approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        downsample_filter_ = approx_voxelgrid;
    } else if(method == "UNIFORM") {
        msg << "[Prefilterer] Downsampling with UNIFORM method, radius: " << downsample_resolution << "\n";
        pcl::UniformSampling<Point3I>::Ptr uniform_sampling(new pcl::UniformSampling<Point3I>());
        uniform_sampling->setRadiusSearch(downsample_resolution);
        downsample_filter_ = uniform_sampling;
    } else if(method == "RANDOM") {
        msg << "[Prefilterer] Downsampling with RANDOM method, sample size: " << sample_size << "\n";
        pcl::RandomSample<Point3I>::Ptr random_sample(new pcl::RandomSample<Point3I>());
        random_sample->setSample(sample_size);
        downsample_filter_ = random_sample;
    } else {
        msg << "[Prefilterer] Downsampling with invalid or NONE method\n";
    }

    if(configuration_.verbose_) {
        std::cout << msg.str();
    }
}

// Configures the outlier removal method used to filter point clouds
void Prefilterer::configure_outlier_remover(const std::string& method, int mean_k, double stddev, double radius, int min_neighbours) {
    std::stringstream msg;

    if(method == "STATISTICAL") {
        msg << "[Prefilterer] Removing outliers with STATISTICAL method, mean and stddev: " << mean_k << ", " << stddev << "\n";
        for(int i = 0; i < 4; i++) {
            pcl::StatisticalOutlierRemoval<Point3I>::Ptr sor(new pcl::StatisticalOutlierRemoval<Point3I>());
            sor->setMeanK(mean_k);
            sor->setStddevMulThresh(stddev);
            outlier_removal_filters_.emplace_back(sor);
        }
    } else if(method == "RADIUS") {
        msg << "[Prefilterer] Removing outliers with RADIUS method, radius and min neighbours: " << radius << ", " << min_neighbours << "\n";
        for(int i = 0; i < 4; i++) {
            pcl::RadiusOutlierRemoval<Point3I>::Ptr ror(new pcl::RadiusOutlierRemoval<Point3I>());
            ror->setRadiusSearch(radius);
            ror->setMinNeighborsInRadius(min_neighbours);
            outlier_removal_filters_.emplace_back(ror);
        }
    } else {
        msg << "[Prefilterer] Removing outliers with invalid or NONE method\n";
    }

    if(configuration_.verbose_) {
        std::cout << msg.str();
    }
}

// Signals that a new raw point cloud has been received
void Prefilterer::update_raw_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) {
    prefilterer_dispatcher_->dispatch([this, pointcloud]{filter_pointcloud(pointcloud);});
}

// Signals that a new IMU datum has been received
void Prefilterer::update_raw_imu_observer(const IMU3D_MSG::ConstPtr& imu3d_msg) {
    std::lock_guard<std::mutex> imu_lock(insertion_mutex_);
    imu3d_msgs_.push_back(imu3d_msg);
}

// Registers an object waiting for filtered point clouds
void Prefilterer::register_filtered_pointcloud_observer(FilteredPointcloudObserver* filtered_pointcloud_observer) {
    filtered_pointcloud_observers_.emplace_back(filtered_pointcloud_observer);
}

// Removes an object waiting for filtered point clouds
void Prefilterer::remove_filtered_pointcloud_observer(FilteredPointcloudObserver* filtered_pointcloud_observer) {
    auto iterator = std::find(filtered_pointcloud_observers_.begin(), filtered_pointcloud_observers_.end(), filtered_pointcloud_observer);

    if(iterator != filtered_pointcloud_observers_.end()) {
        filtered_pointcloud_observers_.erase(iterator);
    }
}

// Notifies all the objects waiting for filtered point clouds
void Prefilterer::notify_filtered_pointcloud_observers(const pcl::PointCloud<Point3I>::ConstPtr& filtered_pointcloud) {
    for(FilteredPointcloudObserver* observer : filtered_pointcloud_observers_) {
        observer->update_filtered_pointcloud_observer(filtered_pointcloud);
    }
}

// Filters and adjusts a point cloud
void Prefilterer::filter_pointcloud(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) {
    auto start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<Point3I>::ConstPtr source_pointcloud = pointcloud->makeShared();
    std::stringstream msg;

    if(configuration_.verbose_) {
        msg << "[Prefilterer] Prefiltering cloud with (id, #points): (" << pointcloud->header.seq << ", " << source_pointcloud->size()
            << ")\n";
        std::cout << msg.str();
        msg.str("");
    }

    if(source_pointcloud->empty()) {
        if(configuration_.verbose_) {
            msg << "[Prefilterer] Empty cloud, skipping the prefiltering\n";
            std::cout << msg.str();
        }
        return;
    }

    source_pointcloud = deskew(source_pointcloud);

    pcl::PointCloud<Point3I>::ConstPtr filtered_pointcloud = distance_filter(source_pointcloud);
    filtered_pointcloud = downsample(filtered_pointcloud);
    filtered_pointcloud = remove_outliers(filtered_pointcloud);

    if(filtered_pointcloud->empty()) {
        msg << "[Prefilterer] Empty cloud after prefiltering!!\n";
        if(configuration_.verbose_) {
            std::cout << msg.str();
        }
        return;
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    total_time_ += duration.count();
    count_++;

    if(configuration_.verbose_) {
        msg << "[Prefilterer] Total time and count: " << total_time_ << ", " << count_ << "\n";
        std::cout << msg.str();
    }

    notify_filtered_pointcloud_observers(filtered_pointcloud);
}

// Deskews the point cloud using IMU data
pcl::PointCloud<Point3I>::ConstPtr Prefilterer::deskew(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) {
    auto start = std::chrono::high_resolution_clock::now();
    std::stringstream msg;

    uint64_t timestamp = pointcloud->header.stamp;

    std::unique_lock<std::mutex> imu_lock(insertion_mutex_);
    if(imu3d_msgs_.empty()) {
        // no possible deskew
        // TODO deskew with no IMU, i.e., motion extracted from consecutive point clouds
        if(configuration_.verbose_) {
            msg << "[Prefilterer] No IMU data available to perform deskewing\n";
            std::cout << msg.str();
        }
        return pointcloud;
    }

    IMU3D_MSG::ConstPtr imu3d_msg = imu3d_msgs_.front();

    // search for the closest IMU datum, in time, to the point cloud
    auto loc = imu3d_msgs_.begin();
    for(; loc != imu3d_msgs_.end(); loc++) {
        // TODO take only IMU within time range
        imu3d_msg = (*loc);
        if((*loc)->header_.timestamp_ > timestamp) {
            break;
        }
    }

    // remove old and useless IMU data and unlock the mutex
    imu3d_msgs_.erase(imu3d_msgs_.begin(), loc);
    imu_lock.unlock();

    EigMatrix3d imu_to_lidar_rotation;
    imu_to_lidar_rotation << configuration_.imu_to_lidar_rotation_[0], configuration_.imu_to_lidar_rotation_[1], configuration_.imu_to_lidar_rotation_[2],
            configuration_.imu_to_lidar_rotation_[3], configuration_.imu_to_lidar_rotation_[4], configuration_.imu_to_lidar_rotation_[5],
            configuration_.imu_to_lidar_rotation_[6], configuration_.imu_to_lidar_rotation_[7], configuration_.imu_to_lidar_rotation_[8];

    // TODO check this operation
    Eigen::Vector3d angular_velocity(imu3d_msg->angular_velocity_.x(), imu3d_msg->angular_velocity_.y(), imu3d_msg->angular_velocity_.z());
    angular_velocity = imu_to_lidar_rotation * angular_velocity;
    angular_velocity *= -1; // multiply by -1 to compensate for the angular motion

    // create a copy of the original point cloud
    pcl::PointCloud<Point3I>::Ptr deskewed(new pcl::PointCloud<Point3I>());
    deskewed->header = pointcloud->header;
    deskewed->is_dense = pointcloud->is_dense;
    deskewed->width = pointcloud->width;
    deskewed->height = pointcloud->height;
    deskewed->resize(pointcloud->size());

    for(int i = 0; i < pointcloud->size(); i++) {
        const auto& pt = pointcloud->at(i);

        // TODO this assumes that the timestamp of the pointcloud = start of the scan
        // here we assume IMU data to be in the same frame as the LiDAR, implying that the conversion
        // was dealth with beforehand; nevertheless, it is just a 3D rotation

        // find the time interval associated to the current point
        float delta_t = configuration_.scan_period_ * static_cast<float>(i) / pointcloud->size();

        // estimate the current angular displacement of the current point
        Eigen::Quaternionf delta_q(1, delta_t / 2.0 * angular_velocity[0], delta_t / 2.0 * angular_velocity[1], delta_t / 2.0 * angular_velocity[2]);
        Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

        // correct the current point
        deskewed->at(i) = pointcloud->at(i);
        deskewed->at(i).getVector3fMap() = pt_;
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    if(configuration_.verbose_) {
        msg << "[Prefilterer] Total deskewing time (ms): " << duration.count() << "\n";
        std::cout << msg.str();
    }

    return deskewed;
}

// Filters the points of the point cloud, within threshold boundaries
pcl::PointCloud<Point3I>::ConstPtr Prefilterer::distance_filter(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) const {
    auto start = std::chrono::high_resolution_clock::now();
    std::stringstream msg;

    // if distance filtering is not enabled, do nothing on the original cloud
    if(!configuration_.use_distance_filter_) {
        return pointcloud;
    }

    pcl::PointCloud<Point3I>::Ptr filtered_pointcloud(new pcl::PointCloud<Point3I>());
    filtered_pointcloud->reserve(pointcloud->size());

    // take only the points within the threshold boundaries
    std::copy_if(pointcloud->begin(), pointcloud->end(), std::back_inserter(filtered_pointcloud->points), [&](const Point3I& p) {
        double d = p.getVector3fMap().norm();
        return d > configuration_.distance_near_threshold_ && d < configuration_.distance_far_threshold_;
    });

    // copy the rest of the original cloud information
    filtered_pointcloud->header = pointcloud->header;
    filtered_pointcloud->width = filtered_pointcloud->size();
    filtered_pointcloud->height = 1;
    filtered_pointcloud->is_dense = false;

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    if(configuration_.verbose_) {
        msg << "[Prefilterer] Total distance filtering time (ms): " << duration.count() << ", size: " << filtered_pointcloud->size() << "\n";
        std::cout << msg.str();
    }

    return filtered_pointcloud;
}

// Downsamples the point cloud to reduce its size
pcl::PointCloud<Point3I>::ConstPtr Prefilterer::downsample(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) {
    auto start = std::chrono::high_resolution_clock::now();
    std::stringstream msg;

    // if downsampling is not enabled, do nothing on the original cloud
    if(!downsample_filter_) {
        return pointcloud;
    }

    pcl::PointCloud<Point3I>::Ptr filtered_pointcloud(new pcl::PointCloud<Point3I>());
    downsample_filter_->setInputCloud(pointcloud);
    downsample_filter_->filter(*filtered_pointcloud);
    filtered_pointcloud->header = pointcloud->header;   // the only information to copy

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    if(configuration_.verbose_) {
        msg << "[Prefilterer] Total downsampling time (ms): " << duration.count() << ", size: " << filtered_pointcloud->size() << "\n";
        std::cout << msg.str();
    }

    return filtered_pointcloud;
}

// Removes noise and outliers from the point cloud
pcl::PointCloud<Point3I>::ConstPtr Prefilterer::remove_outliers(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) {
    auto start = std::chrono::high_resolution_clock::now();
    std::stringstream msg;

    // if outlier removal is not enabled, do nothing on the original cloud
    if(outlier_removal_filters_.empty()) {
        return pointcloud;
    }

    std::vector<pcl::PointCloud<Point3I>::Ptr> partial_pointclouds;
    #pragma omp parallel default(none) shared(pointcloud, std::cout, partial_pointclouds)
    {
        pcl::PointCloud<Point3I>::Ptr private_pointcloud(new pcl::PointCloud<Point3I>());

        #pragma omp for nowait
        for(int i = 0; i < 4; i++) {
            pcl::PassThrough<Point3I> passthrough_filter;

            // first, cut the original point cloud on the x axis
            passthrough_filter.setInputCloud(pointcloud);
            passthrough_filter.setFilterFieldName("x");
            if(i < 2)
                passthrough_filter.setFilterLimits(0, FLT_MAX);
            else
                passthrough_filter.setFilterLimits(-FLT_MAX, 0);
            passthrough_filter.filter(*private_pointcloud);

            // then, cut the filtered point cloud on the y axis
            passthrough_filter.setInputCloud(private_pointcloud);
            passthrough_filter.setFilterFieldName("y");
            if(i == 0 || i == 2)
                passthrough_filter.setFilterLimits(0, FLT_MAX);
            else
                passthrough_filter.setFilterLimits(-FLT_MAX, 0);
            passthrough_filter.filter(*private_pointcloud);

            if(!private_pointcloud->empty()) {
                outlier_removal_filters_[i]->setInputCloud(private_pointcloud);
                outlier_removal_filters_[i]->filter(*private_pointcloud);
            }
        }

        #pragma omp critical
        {
        partial_pointclouds.emplace_back(private_pointcloud);
        }
    }

    // merge the denoised point clouds
    pcl::PointCloud<Point3I>::Ptr denoised_pointcloud(new pcl::PointCloud<Point3I>());
    for(pcl::PointCloud<Point3I>::Ptr& partial_pointcloud : partial_pointclouds) {
        *denoised_pointcloud += *partial_pointcloud;
    }
    denoised_pointcloud->header = pointcloud->header;   // the only information to copy

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    if(configuration_.verbose_) {
        msg << "[Prefilterer] Total outlier removal time (ms): " << duration.count() << ", size: " << denoised_pointcloud->size() << "\n";
        std::cout << msg.str();
    }

    return denoised_pointcloud;
}