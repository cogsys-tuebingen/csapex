#ifndef PORT_TYPE_H
#define PORT_TYPE_H

/// COMPONENT
#include <csapex/csapex_export.h>

/// SYSTEM
#include <string>

namespace csapex
{
enum class ConnectorType {
    OUTPUT, INPUT, SLOT_T, EVENT
};

namespace port_type
{
CSAPEX_EXPORT std::string name(ConnectorType type);

CSAPEX_EXPORT ConnectorType opposite(ConnectorType type);
}
}

#endif // PORT_TYPE_H
