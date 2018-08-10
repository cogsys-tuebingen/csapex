#ifndef REGISTER_PARAMETER_H
#define REGISTER_PARAMETER_H

// COMPONENT
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/param/parameter_factory.h>

#define PARAM_CONCATENATE_DETAIL(x, y) x##y
#define PARAM_CONCATENATE(x, y) PARAM_CONCATENATE_DETAIL(x, y)
#define PARAM_MAKE_UNIQUE(x) PARAM_CONCATENATE(x, __LINE__)

#define CSAPEX_REGISTER_PARAM_WITH_NAME(name, instancename)                                                                                                                                            \
    CSAPEX_REGISTER_PARAMETER_SERIALIZER(name)                                                                                                                                                         \
    namespace csapex                                                                                                                                                                                   \
    {                                                                                                                                                                                                  \
    namespace param                                                                                                                                                                                    \
    {                                                                                                                                                                                                  \
    static factory::ParameterFactory::ParameterConstructorRegistered<name> PARAM_CONCATENATE(c_, instancename);                                                                                        \
    }                                                                                                                                                                                                  \
    }

#define CSAPEX_REGISTER_PARAM(name) CSAPEX_REGISTER_PARAM_WITH_NAME(name, PARAM_MAKE_UNIQUE(g_instance))

#endif  // REGISTER_PARAMETER_H
