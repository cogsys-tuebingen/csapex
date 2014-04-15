#ifndef SET_TIMESTAMP_H_
#define SET_TIMESTAMP_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

/// SYSTEM
#include <QLCDNumber>

namespace csapex {

class SetTimeStamp : public Node
{
    Q_OBJECT

public:
    SetTimeStamp();

    virtual void setup();
    virtual void process();

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
