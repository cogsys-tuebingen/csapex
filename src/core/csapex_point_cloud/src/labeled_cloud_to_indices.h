#ifndef LABELED_CLOUD_TO_INDICES_H
#define LABELED_CLOUD_TO_INDICES_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class LabeledCloudToIndices : public csapex::Node
{
public:
    LabeledCloudToIndices();

    virtual void process();
    virtual void setup();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

protected:
    ConnectorIn*  input_;
    ConnectorOut* output_;
};
}
#endif // LABELED_CLOUD_TO_INDICES_H
