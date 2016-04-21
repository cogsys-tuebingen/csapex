/// HEADER
#include <csapex/utility/slim_signal.hpp>

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/param/param_fwd.h>
#include <yaml-cpp/yaml.h>

namespace csapex
{
namespace slim_signal
{
template class Signal<void()>;
template class Signal<void(bool)>;
template class Signal<void(int)>;
template class Signal<void(unsigned int)>;
template class Signal<void(long)>;
template class Signal<void(unsigned long)>;
template class Signal<void(Connectable*)>;
template class Signal<void(Connectable*,Connectable*)>;
template class Signal<void(Connectable&)>;
template class Signal<void(ConnectablePtr)>;
template class Signal<void(Event*)>;
template class Signal<void(const UUID&)>;
template class Signal<void(const YAML::Node&)>;
template class Signal<void(param::ParameterPtr)>;
}
}
