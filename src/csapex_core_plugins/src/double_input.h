#ifndef DOUBLE_INPUT_H
#define DOUBLE_INPUT_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <QDoubleSpinBox>

namespace csapex {

class DoubleInput : public Node
{
public:
    DoubleInput();

    void tick();
    void setup();
    void process();

private:
    ConnectorOut* out_;
};

}

#endif // DOUBLE_INPUT_H
