#ifndef STATIC_TRANSFORM_H
#define STATIC_TRANSFORM_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class StaticTransform : public csapex::Node
{
    Q_OBJECT

public:
    StaticTransform();

    virtual void setup();

public Q_SLOTS:
    virtual void tick();

private:
    ConnectorOut* output_;
};

}

#endif // STATIC_TRANSFORM_H
