/// HEADER
#include <csapex/model/connectable_vector.h>

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>

using namespace csapex;

template <typename T>
std::vector<ConnectorDescription> ConnectableVector<T>::getDescription() const
{
    std::vector<ConnectorDescription> res;
    for(const std::shared_ptr<T>& c : *this) {
        res.push_back(c->getDescription());
    }
    return res;
}

namespace csapex
{
template class ConnectableVector<Input>;
template class ConnectableVector<Output>;
template class ConnectableVector<Event>;
template class ConnectableVector<Slot>;
}
