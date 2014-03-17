#ifndef LABEL_POINTCLOUD_H
#define LABEL_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class ColorPointCloud : public csapex::Node
{
public:
    ColorPointCloud();

    virtual void process();
    virtual void setup();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

protected:
    ConnectorIn*  input_;
    ConnectorOut* output_;
};
}
#endif // LABEL_POINTCLOUD_H
