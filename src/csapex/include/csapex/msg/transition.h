#ifndef TRANSITION_H
#define TRANSITION_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

namespace csapex
{

class Transition
{
public:
    Transition(NodeWorker* node);

    NodeWorker* getNode() const;

private:
    NodeWorker* node_;
};

}

#endif // TRANSITION_H

