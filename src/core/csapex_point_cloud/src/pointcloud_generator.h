#ifndef POINTCLOUD_GENERATOR_H
#define POINTCLOUD_GENERATOR_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class PointCloudGenerator : public csapex::Node
{
public:
    PointCloudGenerator();

    void process();
    virtual void setup();

private:
    ConnectorOut *out_;
};
}

#endif // POINTCLOUD_GENERATOR_H
