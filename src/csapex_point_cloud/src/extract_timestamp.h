#ifndef EXTRACT_TIMESTAMP_H_
#define EXTRACT_TIMESTAMP_H_

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_point_cloud/point_cloud_message.h>

/// SYSTEM
#include <QLCDNumber>

namespace csapex {

class ExtractTimeStamp : public BoxedObject
{
    Q_OBJECT

public:
    ExtractTimeStamp();

    virtual void fill(QBoxLayout* layout);
    virtual void allConnectorsArrived();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* input_;
    ConnectorOut* output_;
};

}

#endif // EXTRACT_TIMESTAMP_H_
