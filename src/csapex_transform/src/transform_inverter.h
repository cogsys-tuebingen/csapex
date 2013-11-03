#ifndef TRANSFORM_INVERTER_H
#define TRANSFORM_INVERTER_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class TransformInverter : public csapex::Node
{
    Q_OBJECT

public:
    TransformInverter();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    ConnectorIn* input_;
    ConnectorOut* output_;
};

}

#endif // TRANSFORM_INVERTER_H
