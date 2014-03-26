#ifndef CLUSTER_POINTCLOUD_H
#define CLUSTER_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class ClusterPointcloud : public csapex::Node
{
public:
    ClusterPointcloud();

    virtual void process();
    virtual void setup();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* in_cloud_;
    ConnectorOut* out_;
    ConnectorOut* out_debug_;

    double param_clusterTolerance_;
    int param_clusterMinSize_;
    int param_clusterMaxSize_;
};
}
#endif // CLUSTER_POINTCLOUD_H
