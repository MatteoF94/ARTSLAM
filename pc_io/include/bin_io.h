#ifndef ARTSLAM_BIN_IO_H
#define ARTSLAM_BIN_IO_H


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class BinIO {
public:
    typedef pcl::PointXYZI PointT;
    BinIO();

    void read_point_cloud(const std::string& filename, const pcl::PointCloud<PointT>::Ptr& pc);
};


#endif //ARTSLAM_BIN_IO_H
