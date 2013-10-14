#ifndef SET_TIMESTAMP_H_
#define SET_TIMESTAMP_H_

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_point_cloud/point_cloud_message.h>

/// SYSTEM
#include <QLCDNumber>

namespace csapex {

class SetTimeStamp : public BoxedObject
{
    Q_OBJECT

public:
    SetTimeStamp();

    virtual void fill(QBoxLayout* layout);
    virtual void allConnectorsArrived();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* input_;
    ConnectorIn* input_time_;
    ConnectorIn* input_frame_;
    ConnectorOut* output_;
};

}

#endif // SET_TIMESTAMP_H_
