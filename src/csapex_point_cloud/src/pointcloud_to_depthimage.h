#ifndef POINTCLOUD_TO_DEPTHIMAGE_H
#define POINTCLOUD_TO_DEPTHIMAGE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex
{

class PointCloudToDepthImage : public Node
{
public:
    PointCloudToDepthImage();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* input_;
    ConnectorOut* output_;
};

}

#endif // POINTCLOUD_TO_DEPTHIMAGE_H
