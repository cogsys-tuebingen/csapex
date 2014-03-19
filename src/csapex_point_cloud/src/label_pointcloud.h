#ifndef LABEL_POINTCLOUD_H
#define LABEL_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_vision/cv_mat_message.h>

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
    ConnectorIn*                        input_;
    ConnectorIn*                        labels_;
    ConnectorOut*                       output_;
    connection_types::CvMatMessage::Ptr label_msg_;
};
}
#endif // LABEL_POINTCLOUD_H
