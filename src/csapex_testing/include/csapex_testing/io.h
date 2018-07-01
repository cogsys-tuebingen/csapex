#ifndef CSAPEX_TESTING_IO_H
#define CSAPEX_TESTING_IO_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/msg/token_traits.h>
#include <csapex/utility/utility_fwd.h>

namespace csapex
{
namespace testing
{
NodeFacadeImplementationPtr getNodeFacade(const GraphFacadeImplementation& main_facade, const UUID& id);
NodeHandlePtr getNodeHandle(const NodeFacadePtr& node);

InputPtr getInput(const NodeFacadePtr& node, const std::string& name);
OutputPtr getOutput(const NodeFacadePtr& node, const std::string& name);

TokenDataConstPtr getAddedMessage(const OutputPtr& node);

template <typename T>
std::shared_ptr<const T> getAddedMessage(const OutputPtr& output,
                                         typename std::enable_if<!(connection_types::should_use_value_message<T>::value || connection_types::should_use_pointer_message<T>::value)>::type* = 0)
{
    auto data = getAddedMessage(output);
    return std::dynamic_pointer_cast<const T>(data);
}

template <typename T>
std::shared_ptr<const connection_types::GenericValueMessage<T>> getAddedMessage(const OutputPtr& output, typename std::enable_if<connection_types::should_use_value_message<T>::value>::type* = 0)
{
    return getAddedMessage<connection_types::GenericValueMessage<T>>(output);
}
template <typename T>
std::shared_ptr<const connection_types::GenericPointerMessage<T>> getAddedMessage(const OutputPtr& output, typename std::enable_if<connection_types::should_use_pointer_message<T>::value>::type* = 0)
{
    return getAddedMessage<connection_types::GenericPointerMessage<T>>(output);
}
}  // namespace testing

}  // namespace csapex

#endif  // CSAPEX_TESTING_IO_H
