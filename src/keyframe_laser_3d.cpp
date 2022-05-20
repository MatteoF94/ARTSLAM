#include "keyframe_laser_3d.h"

using namespace artslam::laser3d;

KeyframeLaser3D::KeyframeLaser3D(uint64_t timestamp, double accumulated_distance, double reliability, const EigIsometry3d& odometry, const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) : Keyframe(timestamp, accumulated_distance, reliability) {
    odometry_ = odometry;
    pointcloud_ = pointcloud;
    graph_node_ = nullptr;
}

KeyframeLaser3D::~KeyframeLaser3D() = default;

long KeyframeLaser3D::id() const {
    if(graph_node_)
        return graph_node_->id();
    else
        return -1;
}

Eigen::Isometry3d KeyframeLaser3D::estimate() const {
    if(graph_node_)
        return graph_node_->estimate();
    else
        // TODO change in something different than the identity
        return Eigen::Isometry3d::Identity();
}