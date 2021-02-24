/** @file prefilterer.cpp
 * @brief Definition of class Prefilterer
 * @author Matteo Frosi
 */

#include <prefilterer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <chrono>
#include <fstream>

/* ------------------ */
/* Class constructor. */
/* ------------------ */
Prefilterer::Prefilterer() {
    PrefiltererConfig default_config;
    configure_prefilterer(&default_config);
}

/* ----------------------------------- */
/* Class constructor, with parameters. */
/* ----------------------------------- */
Prefilterer::Prefilterer(const PrefiltererConfig *const config_ptr) {
    configure_prefilterer(config_ptr);
}

/* ----------------- */
/* Class destructor. */
/* ----------------- */
Prefilterer::~Prefilterer() {
    std::ostringstream to_print;
    to_print << "[Prefilterer] destroying the prefilterer\n";
    std::cout << to_print.str();
    delete this->prefilterer_dqueue;
    delete this->imu_insertion_dqueue;
}

/* --------------------------------------------------------------- */
/* Configures the prefilterer object using a configuration object. */
/* --------------------------------------------------------------- */
void Prefilterer::configure_prefilterer(const PrefiltererConfig *const config_ptr) {
    std::ostringstream to_print;

    to_print << "[Prefilterer][configure_prefilterer] creating and configurating the prefilterer\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    configure_downsample(config_ptr);
    configure_outlier_removal(config_ptr);

    this->use_distance_filter = config_ptr->use_distance_filter;
    this->distance_near_thresh = config_ptr->distance_near_thresh;
    this->distance_far_thresh = config_ptr->distance_far_thresh;

    this->base_link_frame = "";
    this->pc_to_base_link_transform = Eigen::Matrix4f::Identity(); // default no transform

    this->total_time = 0;

    this->prefilterer_dqueue = new DispatchQueue("prefilterer queue", 1);
    this->imu_insertion_dqueue = new DispatchQueue("prefilterer imu insertion queue", 1);

    to_print << "[Prefilterer][configure_prefilterer] finished creating and configurating the prefilterer\n";
    std::cout << to_print.str();
}

/* --------------------------------------------------------------- */
/* Configures the downsampling method used to filter point clouds. */
/* --------------------------------------------------------------- */
void Prefilterer::configure_downsample(const PrefiltererConfig *const config_ptr) {
    std::string downsample_method = config_ptr->downsample_method;
    double downsample_resolution = config_ptr->downsample_resolution;
    int sample_size = config_ptr->sample_size;

    std::ostringstream to_print;

    if(downsample_method == "VOXELGRID") {
        to_print << "[Prefilterer][configure_downsample] downsample: VOXELGRID, res " << downsample_resolution << "\n";
        boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        this->downsample_filter = voxelgrid;
    } else if (downsample_method == "APPROX_VOXELGRID") {
        to_print << "[Prefilterer][configure_downsample] downsample: APPROX_VOXELGRID, res " << downsample_resolution << "\n";
        boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
        approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        this->downsample_filter = approx_voxelgrid;
    } else if (downsample_method == "UNIFORM") {
        to_print << "[Prefilterer][configure_downsample] downsample: UNIFORM, radius " << downsample_resolution << "\n";
        boost::shared_ptr<pcl::UniformSampling<PointT>> uniform(new pcl::UniformSampling<PointT>());
        uniform->setRadiusSearch(downsample_resolution);
        this->downsample_filter = uniform;
    } else if (downsample_method == "RANDOM") {
        to_print << "[Prefilterer][configure_downsample] downsample: RANDOM, sample size " << sample_size << "\n";
        boost::shared_ptr<pcl::RandomSample<PointT>> random(new pcl::RandomSample<PointT>());
        random->setSample(sample_size);
        this->downsample_filter = random;
    } else {
        if (downsample_method != "NONE") {
            to_print << "[Prefilterer][configure_downsample] downsample: NONE, no downsampling\n";
        }
    }

    std::cout << to_print.str();
}

/* ------------------------------------------------------------------ */
/* Configures the outlier removal method used to filter point clouds. */
/* ------------------------------------------------------------------ */
void Prefilterer::configure_outlier_removal(const PrefiltererConfig *const config_ptr) {
    std::string outlier_removal_method = config_ptr->outlier_removal_method;

    std::ostringstream to_print;

    if(outlier_removal_method == "STATISTICAL") {
        int mean_k = config_ptr->statistical_mean_k;
        double stddev_mul_thresh = config_ptr->statistical_stddev;
        to_print << "[Prefilterer][configure_outlier_removal] outlier_removal: STATISTICAL, mean " << mean_k << " - stddev " << stddev_mul_thresh << "\n";
        for(int i = 0; i < 4; i++) {
            pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
            sor->setMeanK(mean_k);
            sor->setStddevMulThresh(stddev_mul_thresh);
            this->or_filters.emplace_back(sor);
        }
    } else if (outlier_removal_method == "RADIUS") {
        double radius = config_ptr->radius_radius;
        int min_neighbors = config_ptr->radius_min_neighbors;
        to_print << "[Prefilterer][configure_outlier_removal] outlier_removal: RADIUS, radius " << radius << " - min neighbors " << min_neighbors << "\n";
        for(int i = 0; i < 4; i++) {
            pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
            rad->setRadiusSearch(radius);
            rad->setMinNeighborsInRadius(min_neighbors);
            this->or_filters.emplace_back(rad);
        }
    } else {
        if (outlier_removal_method != "NONE") {
            to_print << "[Prefilterer][configure_outlier_removal] outlier_removal: NONE, no outlier removal\n";
        }
    }

    std::cout << to_print.str();
}

/* ------------------------------------------------ */
/* Registers an object waiting for filtered clouds. */
/* ------------------------------------------------ */
void Prefilterer::register_filtered_cloud_observer(FilteredCloudObserver *const fc_observer_ptr_tmp) {
    this->filtered_cloud_observers.emplace_back(fc_observer_ptr_tmp);
}

/* ---------------------------------------------- */
/* Removes an object waiting for filtered clouds. */
/* ---------------------------------------------- */
void Prefilterer::remove_filtered_cloud_observer(FilteredCloudObserver *const fc_observer_ptr_tmp) {
    auto iterator = std::find(this->filtered_cloud_observers.begin(), this->filtered_cloud_observers.end(), fc_observer_ptr_tmp);

    if(iterator != this->filtered_cloud_observers.end()) {
        this->filtered_cloud_observers.erase(iterator);
    }
}

/* ----------------------------------------------------- */
/* Notifies all the observers with a new filtered cloud. */
/* ----------------------------------------------------- */
void Prefilterer::notify_filtered_cloud_observers(const pcl::PointCloud<PointT>::ConstPtr& filtered_cloud) {
    for(FilteredCloudObserver *observer : this->filtered_cloud_observers) {
        observer->update(filtered_cloud);
    }
}

/* ----------------------------------------------------- */
/* Signals that a new raw point cloud has been received. */
/* ----------------------------------------------------- */
void Prefilterer::update(pcl::PointCloud<PointT>::ConstPtr src_cloud) {
    dispatch_filter_cloud(src_cloud);
}

/* -------------------------------------------- */
/* Signals that new IMU data has been received. */
/* -------------------------------------------- */
void Prefilterer::update(const ImuMSG::ConstPtr &imu_msg_constptr) {
    dispatch_imu_insertion(imu_msg_constptr);
}

/* ---------------------------------- */
/* Filters and adjusts a point cloud. */
/* ---------------------------------- */
void Prefilterer::filter_cloud(pcl::PointCloud<PointT>::ConstPtr src_cloud) {
    auto start = std::chrono::high_resolution_clock::now();
    int seq_id = src_cloud->header.seq;
    std::ostringstream to_print;
    to_print << "[Prefilterer][filter_cloud][seq:" << seq_id << "] filtering " << src_cloud->points.size() << " points\n";

    if(src_cloud->empty()) {
        to_print << "[Prefilterer][filter_cloud][seq:" << seq_id << "] empty cloud to prefilter, skipping";
        std::cout << to_print.str();
        return;
    }

    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    // deskewing with IMU data
    // TODO correct also positions
    src_cloud = deskewing(src_cloud);

    // transform the cloud w.r.t. the base link frame, if exist, otherwise no transformation is applied
    if (!this->base_link_frame.empty()) {
        pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*src_cloud, *transformed, this->pc_to_base_link_transform);

        transformed->header.frame_id = this->base_link_frame;
        transformed->header.stamp = src_cloud->header.stamp;
        transformed->header.seq = src_cloud->header.seq;

        src_cloud = transformed;
    }

    // filtering
    pcl::PointCloud<PointT>::ConstPtr filtered_cloud = this->use_distance_filter ? distance_filter(src_cloud) : src_cloud;
    filtered_cloud = downsample(filtered_cloud);
    filtered_cloud = outlier_removal(filtered_cloud);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    this->times.emplace_back(duration.count());
    this->total_time += duration.count();
    to_print << "[Prefilterer][filter_cloud][seq:" << seq_id << "] finished filtering, (before - after) " << src_cloud->points.size() << " - " << filtered_cloud->points.size() << ", time: " << duration.count() << "\n";
    std::cout << to_print.str();

    // notifies all objects waiting for a filtered point cloud
    notify_filtered_cloud_observers(filtered_cloud);
}

/* -------------------------------------------- */
/* Deskews a point cloud using IMU information. */
/* -------------------------------------------- */
pcl::PointCloud<PointT>::ConstPtr Prefilterer::deskewing(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    int seq_id = cloud->header.seq;
    std::ostringstream to_print;
    to_print << "[Prefilterer][deskewing][seq:" << seq_id << "] deskewing\n";

    std::lock_guard<std::mutex> lock(this->imu_insertion_mutex);
    if(imu_queue.empty()) {
        to_print << "[Prefilterer][deskewing][seq:" << seq_id << "] no imu data, no deskewing\n";
        std::cout << to_print.str();
        return cloud;
    }

    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    ImuMSG::ConstPtr imu_msg = this->imu_queue.front();

    // search for the closest IMU datum
    auto loc = this->imu_queue.begin();
    for(; loc != this->imu_queue.end(); loc++) {
        imu_msg = *loc;
        if(imu_msg->header.timestamp > cloud->header.stamp) {
            break;
        }
    }

    // delete the previous data to enforce temporal coherence
    this->imu_queue.erase(this->imu_queue.begin(), loc);

    Eigen::Vector3d angular_velocity(imu_msg->angular_velocity);
    angular_velocity *= -1;

    pcl::PointCloud<PointT>::Ptr deskewd_cloud(new pcl::PointCloud<PointT>());
    deskewd_cloud->header = cloud->header;
    deskewd_cloud->is_dense = cloud->is_dense;
    deskewd_cloud->width = cloud->width;
    deskewd_cloud->height = cloud->height;
    deskewd_cloud->resize(cloud->size());

    for(int i = 0; i < cloud->size(); i++) {
        const PointT& pt = cloud->at(i);

        double delta_t = this->scan_period * static_cast<double>(i) / cloud->size();
        Eigen::Quaternionf delta_q(1,
                                   delta_t / 2.0 * angular_velocity[0],
                                   delta_t / 2.0 * angular_velocity[1],
                                   delta_t / 2.0 * angular_velocity[2]
                                   );
        Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

        deskewd_cloud->at(i) = cloud->at(i);
        deskewd_cloud->at(i).getVector3fMap() = pt_;
    }

    to_print << "[Prefilterer][deskewing][seq:" << seq_id << "] finished deskewing\n";
    std::cout << to_print.str();
}

/* ---------------------------------------------------- */
/* Filters a point cloud using the range of its points. */
/* ---------------------------------------------------- */
pcl::PointCloud<PointT>::ConstPtr Prefilterer::distance_filter(const pcl::PointCloud<PointT>::ConstPtr &cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());
    int seq_id = cloud->header.seq;
    std::ostringstream to_print;
    to_print << "[Prefilterer][distance_filter][seq:" << seq_id << "] filtering close and distant points\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
        double d = p.getVector3fMap().norm();
        return d > distance_near_thresh && d < distance_far_thresh;
    });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;
    filtered->header = cloud->header;

    to_print << "[Prefilterer][distance_filter][seq:" << seq_id << "] finished distance filtering\n";
    std::cout << to_print.str();

    return filtered;
}

/* -------------------------- */
/* Downsamples a point cloud. */
/* -------------------------- */
pcl::PointCloud<PointT>::ConstPtr Prefilterer::downsample(const pcl::PointCloud<PointT>::ConstPtr &cloud) const {
    int seq_id = cloud->header.seq;
    std::ostringstream to_print;
    to_print << "[Prefilterer][downsample][seq:" << seq_id << "] downsampling\n";

    if(!this->downsample_filter) {
        to_print << "[Prefilterer][downsample][seq:" << seq_id << "] no downsampling method assigned, skipping\n";
        std::cout << to_print.str();
        return cloud;
    }

    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    pcl::PointCloud<PointT>::Ptr downsampled(new pcl::PointCloud<PointT>());
    this->downsample_filter->setInputCloud(cloud);
    this->downsample_filter->filter(*downsampled);
    downsampled->header = cloud->header;

    to_print << "[Prefilterer][downsample][seq:" << seq_id << "] finished downsampling\n";
    std::cout << to_print.str();

    return downsampled;
}

/* ---------------------------------------- */
/* Removes the outliers from a point cloud. */
/* ---------------------------------------- */
pcl::PointCloud<PointT>::ConstPtr Prefilterer::outlier_removal(const pcl::PointCloud<PointT>::ConstPtr &cloud) const {
    int seq_id = cloud->header.seq;
    std::ostringstream to_print;
    to_print << "[Prefilterer][outlier_removal][seq:" << seq_id << "] outlier removal\n";

    if(this->or_filters.empty()) {
        to_print << "[Prefilterer][outlier_removal][seq:" << seq_id << "] no outlier removal assigned, skipping\n";
        std::cout << to_print.str();
        return cloud;
    }

    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    // performs parallel outlier removal on pairs of octants
    std::vector<pcl::PointCloud<PointT>::Ptr> partial_clouds;
    #pragma omp parallel default(none) shared(cloud, std::cout, partial_clouds)
    {
        pcl::PointCloud<PointT>::Ptr private_cloud(new pcl::PointCloud<PointT>());

        #pragma omp for nowait
        for(int i = 0; i < 4; i++){
            pcl::PassThrough<PointT> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            if(i < 2)
                pass.setFilterLimits(0, FLT_MAX);
            else
                pass.setFilterLimits(-FLT_MAX, 0);
            pass.filter(*private_cloud);

            pass.setInputCloud(private_cloud);
            pass.setFilterFieldName("y");
            if(i == 0 || i == 2)
                pass.setFilterLimits(0, FLT_MAX);
            else
                pass.setFilterLimits(-FLT_MAX, 0);
            pass.filter(*private_cloud);

            if(!private_cloud->empty()) {
                this->or_filters[i]->setInputCloud(private_cloud);
                this->or_filters[i]->filter(*private_cloud);
            }
        }

        #pragma omp critical
        partial_clouds.emplace_back(private_cloud);
    }

    pcl::PointCloud<PointT>::Ptr no_outlier_cloud(new pcl::PointCloud<PointT>());
    for(auto & partial_cloud : partial_clouds) {
        *no_outlier_cloud += *partial_cloud;
    }
    no_outlier_cloud->header = cloud->header;

    to_print << "[Prefilterer][outlier_removal][seq:" << seq_id << "] finished outlier removal\n";
    std::cout << to_print.str();

    return no_outlier_cloud;
}

/* --------------------------------------------------------------------------------- */
/* Tells the prefilterer that a point cloud is ready to be dispatched for filtering. */
/* --------------------------------------------------------------------------------- */
void Prefilterer::dispatch_filter_cloud(pcl::PointCloud<PointT>::ConstPtr &cloud) {
    this->prefilterer_dqueue->dispatch([this, cloud]{this->filter_cloud(cloud);});
}

/* -------------------------------------------------------------------------------- */
/* Tells the prefilterer that new IMU data is ready to be dispatched for insertion. */
/* -------------------------------------------------------------------------------- */
void Prefilterer::dispatch_imu_insertion(const ImuMSG::ConstPtr& imu_msg_constptr) {
    this->imu_insertion_dqueue->dispatch([this, imu_msg_constptr]{this->add_imu_measurement(imu_msg_constptr);});
}

/* ----------------------------------------------------------------- */
/* Adds an IMU measurement, to be used when deskewing a point cloud. */
/* ----------------------------------------------------------------- */
void Prefilterer::add_imu_measurement(const ImuMSG::ConstPtr& imu_msg_constptr) {
    std::lock_guard<std::mutex> lock(this->imu_insertion_mutex);
    this->imu_queue.emplace_back(imu_msg_constptr);
}

/* ------------------------------------------------------- */
/* Saves the prefiltering times to file, for benchmarking. */
/* ------------------------------------------------------- */
void Prefilterer::dump_times(const std::string& filename) {
    std::ofstream output_file(filename);

    std::ostream_iterator<uint64_t> output_iterator(output_file, "\n");
    std::copy(this->times.begin(), this->times.end(), output_iterator);
}