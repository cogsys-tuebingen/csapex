#ifndef REGISTER_NODE_ADAPTER_H
#define REGISTER_NODE_ADAPTER_H

/// COMPONENT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/node_adapter_builder.h>

#define CSAPEX_REGISTER_NODE_ADAPTER_NS(Namespace, Adapter, Adaptee) \
namespace Namespace {\
class Adapter##Builder : public csapex::NodeAdapterBuilder \
{ \
public: \
    virtual std::string getWrappedType() const \
    { \
        return #Adaptee; \
    } \
    virtual NodeAdapterPtr build(NodePtr node, WidgetController* widget_ctrl) const \
    { \
        Adaptee* adaptee = dynamic_cast<Adaptee*> (node.get()); \
        return NodeAdapter::Ptr(new Adapter(adaptee, widget_ctrl)); \
    } \
}; \
}\
CSAPEX_REGISTER_CLASS(Namespace::Adapter##Builder, csapex::NodeAdapterBuilder)

#define CSAPEX_REGISTER_NODE_ADAPTER(Adapter, Adaptee) \
CSAPEX_REGISTER_NODE_ADAPTER_NS(csapex, Adapter, Adaptee)

#endif // REGISTER_NODE_ADAPTER_H