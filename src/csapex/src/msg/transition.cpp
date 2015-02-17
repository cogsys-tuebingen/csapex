/// HEADER
#include <csapex/msg/transition.h>

using namespace csapex;

Transition::Transition(NodeWorker* node)
    : node_(node)
{

}

NodeWorker* Transition::getNode() const
{
    return node_;
}
