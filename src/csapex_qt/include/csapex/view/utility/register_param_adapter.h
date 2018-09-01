#ifndef REGISTER_PARAM_ADAPTER_H
#define REGISTER_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/param/param_adapter_factory.h>
#include <csapex/view/param/param_adapter_builder.h>

#define MAKE_PA_CLASS(C) C##Builder
#define MAKE_NS_PA_CLASS(NS, C) NS::MAKE_PA_CLASS(C)
#define PARAM_CONCATENATE_DETAIL(x, y) x##y
#define PARAM_CONCATENATE(x, y) PARAM_CONCATENATE_DETAIL(x, y)
#define PARAM_MAKE_UNIQUE(x) PARAM_CONCATENATE(x, __LINE__)

/// ParamAdapter registration
#define CSAPEX_REGISTER_PARAM_ADAPTER(Namespace, Adapter, Adaptee)                                                                                                                                     \
    namespace Namespace                                                                                                                                                                                \
    {                                                                                                                                                                                                  \
    class MAKE_PA_CLASS(Adapter) : public impl::ParameterAdapterBuilderImplementation<Adapter, Adaptee>                                                                                                \
    {                                                                                                                                                                                                  \
    public:                                                                                                                                                                                            \
        MAKE_PA_CLASS(Adapter)() = default;                                                                                                                                                            \
        const std::type_info& getWrappedTypeId() const override                                                                                                                                        \
        {                                                                                                                                                                                              \
            return typeid(Adaptee);                                                                                                                                                                    \
        }                                                                                                                                                                                              \
    };                                                                                                                                                                                                 \
    }                                                                                                                                                                                                  \
    CSAPEX_REGISTER_CLASS(MAKE_NS_PA_CLASS(Namespace, Adapter), csapex::ParameterAdapterBuilder)                                                                                                       \
    static ParameterAdapterRegistered<MAKE_NS_PA_CLASS(Namespace, Adapter), Adaptee> PARAM_CONCATENATE(c_, PARAM_MAKE_UNIQUE(g_instance));

#endif  // REGISTER_PARAM_ADAPTER_H
