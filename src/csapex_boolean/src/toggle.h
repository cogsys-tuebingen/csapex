#ifndef TOGGLE_H
#define TOGGLE_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <QPushButton>

namespace csapex {

namespace boolean {

class Toggle : public Node
{
public:
    Toggle();

public:
    virtual void setup();
    void process();

    void tick();

private:
    void setSignal();

private:
    ConnectorOut* out;
    bool signal_;
};

}

}

#endif // TOGGLE_H
