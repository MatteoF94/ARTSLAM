#include <bin_io.h>

BinIO::BinIO() {}

void BinIO::read_point_cloud(const std::string& filename, const pcl::PointCloud<PointT>::Ptr& pc) {
    std::fstream input(filename.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "could not read file: " << filename << std::endl;
        return;
    }
    input.seekg(0, std::ios::beg);

    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
        PointT point;
        input.read((char *) &point.x, sizeof(float));
        input.read((char *) &point.y, sizeof(float));
        input.read((char *) &point.z, sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        pc->push_back(point);
    }
    input.close();

    std::cout << "read KITTI point cloud with " << pc->points.size() << " points, from file: " << filename << std::endl;
    std::cout << pc->header << std::endl;
    std::cout << pc->is_dense << std::endl;
    std::cout << pc->width << std::endl;
    std::cout << pc->height << std::endl;
}
