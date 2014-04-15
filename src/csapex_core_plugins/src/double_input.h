#ifndef DOUBLE_INPUT_H
#define DOUBLE_INPUT_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <QDoubleSpinBox>

namespace csapex {

template <typename T>
class NumberInput : public Node
{
public:
    NumberInput();

    void tick();
    void setup();
    void process();

    virtual QIcon getIcon() const;

private:
    ConnectorOut* out_;
};

}

#endif // DOUBLE_INPUT_H
