#ifndef TIME_OFFSET_H_
#define TIME_OFFSET_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/ros_handler.h>
#include <csapex/utility/qdouble_slider.h>

namespace csapex {

class TimeOffset : public csapex::Node
{
public:
    TimeOffset();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    ConnectorOut* output_;
    ConnectorIn* input_;
};

}

#endif // TIME_OFFSET_H_
