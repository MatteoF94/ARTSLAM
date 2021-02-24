/** @file cloud_observer.h
 * @brief Declaration of class CloudObserver
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_CLOUD_OBSERVER_H
#define ARTSLAM_CLOUD_OBSERVER_H


#include <slam_types.h>
#include <pcl/point_cloud.h>

/**
 * @class CloudObserver
 * @brief Interface for objects waiting for raw point clouds (unfiltered), e.g. prefilters or visualizers.
 */
class CloudObserver {
public:
    /**
     * @brief Virtual update method.
     * @param src_cloud The raw point cloud used to update the concrete object.
     */
    virtual void update(pcl::PointCloud<PointT>::ConstPtr src_cloud) = 0;
};


#endif //ARTSLAM_CLOUD_OBSERVER_H
