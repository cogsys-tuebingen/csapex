#ifndef PORT_TYPE_H
#define PORT_TYPE_H

/// COMPONENT
#include <csapex_core/csapex_core_export.h>

/// SYSTEM
#include <string>

namespace csapex
{
enum class ConnectorType
{
    NONE,
    OUTPUT,
    INPUT,
    SLOT_T,
    EVENT
};

namespace port_type
{
CSAPEX_CORE_EXPORT std::string name(ConnectorType type);

CSAPEX_CORE_EXPORT ConnectorType opposite(ConnectorType type);
}  // namespace port_type
}  // namespace csapex

#endif  // PORT_TYPE_H
