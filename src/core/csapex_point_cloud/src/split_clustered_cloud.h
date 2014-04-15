#ifndef SPLIT_CLUSTERED_CLOUD_H
#define SPLIT_CLUSTERED_CLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class SplitClusteredCloud : public csapex::Node
{
public:
    SplitClusteredCloud();

    virtual void process();
    virtual void setup();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

protected:
    ConnectorIn*  input_;
    ConnectorIn* in_indices_;
    ConnectorOut* output1_;
    ConnectorOut* output2_;
    ConnectorOut* output3_;
    ConnectorOut* output4_;
};
}
#endif // SPLIT_CLUSTERED_CLOUD_H
