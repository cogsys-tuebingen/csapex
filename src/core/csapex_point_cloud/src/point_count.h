#ifndef POINT_COUNT_H
#define POINT_COUNT_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_point_cloud/point_cloud_message.h>

/// SYSTEM
#include <QLCDNumber>

namespace csapex {

class PointCount : public BoxedObject
{
    Q_OBJECT

public:
    PointCount();

    virtual void fill(QBoxLayout* layout);
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* input_;

    QLCDNumber* number_;
};

}

#endif // POINT_COUNT_H
