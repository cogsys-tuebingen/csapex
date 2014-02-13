#ifndef TO_POINT_MATRIX_H
#define TO_POINT_MATRIX_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class PointCloutToPointMatrix : public csapex::Node
{
public:
    PointCloutToPointMatrix();

    virtual void allConnectorsArrived();
    virtual void setup();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* input_;
    ConnectorOut* output_;
};
}
#endif // TO_POINT_MATRIX_H
