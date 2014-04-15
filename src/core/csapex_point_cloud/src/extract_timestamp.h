#ifndef EXTRACT_TIMESTAMP_H_
#define EXTRACT_TIMESTAMP_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {

class ExtractTimeStamp : public Node
{
    Q_OBJECT

public:
    ExtractTimeStamp();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* input_;
    ConnectorOut* output_;
    ConnectorOut* output_frame_;
};

}

#endif // EXTRACT_TIMESTAMP_H_
