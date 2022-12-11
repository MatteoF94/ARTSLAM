#ifndef ARTSLAM_3D_LASER_KEYFRAME_LASER_3D_H
#define ARTSLAM_3D_LASER_KEYFRAME_LASER_3D_H

#include <artslam_types/types_pcl.hpp>
#include <artslam_types/types_eigen.hpp>
#include <artslam_types/keyframe.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <g2o/types/slam3d/vertex_se3.h>

using namespace artslam::core::types;

namespace artslam::laser3d {
    class KeyframeLaser3D : public Keyframe {
    public:
        using Ptr = std::shared_ptr<KeyframeLaser3D>;

        using ConstPtr = std::shared_ptr<KeyframeLaser3D>;

        KeyframeLaser3D(uint64_t timestamp, double accumulated_distance, double reliability, const EigIsometry3d& odometry, const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        ~KeyframeLaser3D() override;

        long id() const;

        Eigen::Isometry3d estimate() const;

        // TODO implement save and write

    public:
        // ----------------------------------------------------------------------------------
        // ---------------------------- PARAMETERS AND VARIABLES ----------------------------
        // ----------------------------------------------------------------------------------
        EigIsometry3d odometry_;
        pcl::PointCloud<Point3I>::ConstPtr pointcloud_;
        pcl::PointCloud<Point3I>::Ptr pointcloud_2d_;
        std::optional<EigVector4d> floor_coefficients_;
        std::optional<EigVector3d> lidar_enu_coordinates_;
        std::optional<EigVector3d> lidar_utm_coordinates_;
        std::optional<EigVector3d> gps_utm_coordinates_;
        std::optional<EigVector3d> acceleration_;
        std::optional<EigQuaterniond> orientation_;
        g2o::VertexSE3* graph_node_;
        bool utm_compensated_ = false;
    };
}


#endif //ARTSLAM_3D_LASER_KEYFRAME_LASER_3D_H
