#ifndef PORT_TYPE_H
#define PORT_TYPE_H

/// SYSTEM
#include <string>

namespace csapex
{
enum class ConnectorType {
    OUTPUT, INPUT, SLOT_T, EVENT
};

namespace port_type
{
std::string name(ConnectorType type);

ConnectorType opposite(ConnectorType type);
}
}

#endif // PORT_TYPE_H
