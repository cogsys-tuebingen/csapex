#ifndef REGISTER_NODE_ADAPTER_H
#define REGISTER_NODE_ADAPTER_H

/// COMPONENT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/node/node_adapter_builder.h>

#define MAKE_CLASS(C) C##Builder
#define MAKE_NS_CLASS(NS, C) NS::MAKE_CLASS(C)

/// LocalNodeAdapter registration
#define CSAPEX_REGISTER_LOCAL_NODE_ADAPTER_NS(Namespace, Adapter, Adaptee) \
namespace Namespace { \
class MAKE_CLASS(Adapter) : public impl::LocalAdapterBuilder<Adapter, Adaptee> {}; \
} \
CSAPEX_REGISTER_CLASS(MAKE_NS_CLASS(Namespace,Adapter),csapex::NodeAdapterBuilder)

#define CSAPEX_REGISTER_LOCAL_NODE_ADAPTER(Adapter, Adaptee) \
CSAPEX_REGISTER_LOCAL_NODE_ADAPTER_NS(csapex,Adapter,Adaptee)


/// RemoteNodeAdapter registration
#define CSAPEX_REGISTER_REMOTE_NODE_ADAPTER_NS(Namespace, Adapter, Adaptee) \
namespace Namespace { \
class MAKE_CLASS(Adapter) : public impl::RemoteAdapterBuilder<Adapter> {\
    public: MAKE_CLASS(Adapter)() : impl::RemoteAdapterBuilder<Adapter>(#Adaptee) {} \
}; \
} \
CSAPEX_REGISTER_CLASS(MAKE_NS_CLASS(Namespace,Adapter),csapex::NodeAdapterBuilder)

#define CSAPEX_REGISTER_REMOTE_NODE_ADAPTER(Adapter, Adaptee) \
CSAPEX_REGISTER_REMOTE_NODE_ADAPTER_NS(csapex,Adapter,Adaptee)


#endif // REGISTER_NODE_ADAPTER_H
