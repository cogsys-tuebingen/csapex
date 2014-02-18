#ifndef POINT_COUNT_H
#define POINT_COUNT_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {

class TransformCloud : public BoxedObject
{
    Q_OBJECT

public:
    TransformCloud();

    virtual void fill(QBoxLayout* layout);
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
