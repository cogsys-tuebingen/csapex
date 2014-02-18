#ifndef TRANSFORM_PUBLISHER_H
#define TRANSFORM_PUBLISHER_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <tf/transform_broadcaster.h>

namespace csapex {

class TransformPublisher : public csapex::Node
{
    Q_OBJECT

public:
    TransformPublisher();
    ~TransformPublisher();

    virtual void process();
    virtual void setup();

private:
    ConnectorIn* input_transform;
    ConnectorIn* input_time;

    tf::TransformBroadcaster* tfb_;
};

}

#endif // TRANSFORM_PUBLISHER_H
