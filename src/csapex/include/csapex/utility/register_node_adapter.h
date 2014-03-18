#ifndef REGISTER_NODE_ADAPTER_H
#define REGISTER_NODE_ADAPTER_H

/// COMPONENT
#include <csapex/utility/register_apex_plugin.h>

#define CSAPEX_REGISTER_NODE_ADAPTER(Adapter, Adaptee) \
namespace csapex \
{ \
class OutputDisplayAdapterBuilder : public NodeAdapterBuilder \
{ \
public: \
    virtual std::string getWrappedType() const \
    { \
        return #Adaptee; \
    } \
    virtual NodeAdapterPtr build(NodePtr node) const \
    { \
        Adaptee* adaptee = dynamic_cast<Adaptee*> (node.get()); \
        return NodeAdapter::Ptr(new Adapter(adaptee)); \
    } \
}; \
} \
CSAPEX_REGISTER_CLASS(Adapter##Builder, csapex::NodeAdapterBuilder)


#endif // REGISTER_NODE_ADAPTER_H
