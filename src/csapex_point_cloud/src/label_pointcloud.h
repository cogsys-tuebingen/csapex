#ifndef LABEL_POINTCLOUD_H
#define LABEL_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <opencv2/core/core.hpp>

namespace csapex {
class LabelPointCloud : public csapex::Node
{
public:
    LabelPointCloud();

    virtual void process();
    virtual void setup();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

protected:
    ConnectorIn*  input_;
    ConnectorIn*  labels_;
    ConnectorOut* output_;
    cv::Mat       label_mat_;
};
}
#endif // LABEL_POINTCLOUD_H
