#ifndef REGISTER_NODE_ADAPTER_H
#define REGISTER_NODE_ADAPTER_H

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/node/node_adapter_builder.h>

#define MAKE_CLASS(NS, C) NS::C##Builder
#define CSAPEX_REGISTER_NODE_ADAPTER_NS(Namespace, Adapter, Adaptee) \
namespace Namespace {\
class Adapter##Builder : public csapex::NodeAdapterBuilder \
{ \
public: \
    virtual std::string getWrappedType() const \
    { \
        return #Adaptee; \
    } \
    virtual csapex::NodeAdapterPtr build(csapex::NodeHandlePtr handle, NodeBox* parent) const \
    { \
        std::weak_ptr<Adaptee> adaptee = std::dynamic_pointer_cast<Adaptee> (handle->getNode().lock()); \
        return std::make_shared<Adapter>(handle, parent, adaptee); \
    } \
}; \
}\
CSAPEX_REGISTER_CLASS(MAKE_CLASS(Namespace,Adapter),csapex::NodeAdapterBuilder)

#define CSAPEX_REGISTER_NODE_ADAPTER(Adapter, Adaptee) \
CSAPEX_REGISTER_NODE_ADAPTER_NS(csapex,Adapter,Adaptee)

#endif // REGISTER_NODE_ADAPTER_H
