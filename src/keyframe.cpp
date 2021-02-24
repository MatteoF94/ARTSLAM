/** @file keyframe.cpp
 * @brief Definition of class Keyframe
 * @author Matteo Frosi
*/

#include <keyframe.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

/* ----------------------------------- */
/* Class constructor, with parameters. */
/* ----------------------------------- */
Keyframe::Keyframe(const uint64_t new_timestamp, const Eigen::Isometry3d& new_odom, double new_accum_distance,
                   const pcl::PointCloud<PointT>::ConstPtr& new_cloud) {
    this->timestamp = new_timestamp;
    this->odom = new_odom;
    this->accum_distance = new_accum_distance;
    this->cloud = new_cloud;
    this->fitness_score = 0.0;
    this->node = nullptr;
}

/* ----------------- */
/* Class destructor. */
/* ----------------- */
Keyframe::~Keyframe() = default;

/* --------------------------------------- */
/* Saves the keyframe data in a directory. */
/* --------------------------------------- */
void Keyframe::save_to_dir(const std::string& directory) {
    if(!boost::filesystem::is_directory(directory)) {
        boost::filesystem::create_directory(directory);
    }

    // save general data of the keyframe
    std::ofstream ofs(directory + "/data.txt");
    ofs << "timestamp " << this->timestamp << "\n";

    ofs << "estimate\n";
    ofs << node->estimate().matrix() << "\n";

    ofs << "odom\n";
    ofs << this->odom.matrix() << "\n";

    ofs << "accum_distance " << this->accum_distance << "\n";

    if(this->floor_coeffs) {
        ofs << "floor_coeffs " << this->floor_coeffs->transpose() << "\n";
    }

    if(this->utm_coords) {
        ofs << "utm_coord " << this->utm_coords->transpose() << "\n";
    }

    if(this->acceleration) {
        ofs << "acceleration " << this->acceleration->transpose() << "\n";
    }

    if(this->orientation) {
        ofs << "orientation " << this->orientation->w() << " " << this->orientation->x() << " " << this->orientation->y() << " " << this->orientation->z() << "\n";
    }

    // save the pose estimates in kitti format
    Eigen::Matrix4d roto = Eigen::Matrix4d::Zero();
    roto(3,3) = 1;
    roto(0,1) = -1;
    roto(1,2) = -1;
    roto(2,0) = 1;
    Eigen::Matrix4d estim = this->node->estimate().matrix();
    estim = roto * estim;
    std::ofstream outp;
    outp.open(directory + "/../kitti_format.txt", std::ios_base::app);
    outp << estim(0,0) << " " << estim(0,1) << " " << estim(0,2) << " " << estim(0,3) << " " <<
         estim(1,0) << " " << estim(1,1) << " " << estim(1,2) << " " << estim(1,3) << " " <<
         estim(2,0) << " " << estim(2,1) << " " << estim(2,2) << " " << estim(2,3) << " " << "\n";
    outp.close();

    // save the point cloud in .pcd format
    pcl::io::savePCDFileBinary(directory + "/cloud.pcd", *this->cloud);
}

/* ------------------------------------------------------------------------ */
/* Gets the keyframe ID (actually, the ID of the g2o node of the keyframe). */
/* ------------------------------------------------------------------------ */
long Keyframe::id() const {
    return this->node->id();
}

/* ------------------------------------------------------ */
/* Gets the estimated and optimized pose of the keyframe. */
/* ------------------------------------------------------ */
Eigen::Isometry3d Keyframe::estimate() const {
    return this->node->estimate();
}