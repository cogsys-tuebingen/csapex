#ifndef POINT_COUNT_H
#define POINT_COUNT_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {

class TransformCloud : public Node
{
    Q_OBJECT

public:
    TransformCloud();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* input_cloud_;
    ConnectorIn* input_transform_;
    ConnectorOut* output_;
};

}

#endif // POINT_COUNT_H
