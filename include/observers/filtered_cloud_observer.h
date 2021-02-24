/** @file filtered_cloud_observer.h
 * @brief Declaration of class FilteredCloudObserver
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_FILTERED_CLOUD_OBSERVER_H
#define ARTSLAM_FILTERED_CLOUD_OBSERVER_H


#include <slam_types.h>
#include <pcl/point_cloud.h>

/**
 * @class FilteredCloudObserver
 * @brief Interface for objects waiting for filtered point clouds, e.g. trackers, visualizers or floor detectors.
 */
class FilteredCloudObserver {
public:
    /**
     * @brief Virtual update method.
     * @param src_cloud The filtered point cloud used to update the concrete object.
     */
    virtual void update(pcl::PointCloud<PointT>::ConstPtr src_cloud) = 0;
};


#endif //ARTSLAM_FILTERED_CLOUD_OBSERVER_H
