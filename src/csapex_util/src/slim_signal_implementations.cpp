/// HEADER
#include <csapex/utility/slim_signal.hpp>

/// PROJECT
#include <csapex/utility/uuid.h>
#include <csapex/utility/yaml.h>

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
template class Signal<void(const UUID&)>;
template class Signal<void(const YAML::Node&)>;
}  // namespace slim_signal
}  // namespace csapex
