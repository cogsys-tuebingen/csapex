#ifndef FOREACH_H
#define FOREACH_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class Foreach : public csapex::Node
{
public:
    Foreach();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    ConnectorIn* input_;
    ConnectorOut* output_;
};

}

#endif // FOREACH_H
