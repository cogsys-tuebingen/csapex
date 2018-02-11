/// HEADER
#include <csapex/model/connector_type.h>

/// SYSTEM
#include <stdexcept>

using namespace csapex;

namespace csapex
{
namespace port_type
{

std::string name(ConnectorType type) {
    switch (type) {
    case ConnectorType::OUTPUT:
        return "output";
    case ConnectorType::INPUT:
        return "input";
    case ConnectorType::SLOT_T:
        return "slot";
    case ConnectorType::EVENT:
        return "event";
    default:
        throw std::logic_error("unknown connector type");
    }
}

ConnectorType opposite(ConnectorType type)
{
    switch (type) {
    case ConnectorType::OUTPUT:
        return ConnectorType::INPUT;
    case ConnectorType::INPUT:
        return ConnectorType::OUTPUT;
    case ConnectorType::SLOT_T:
        return ConnectorType::EVENT;
    case ConnectorType::EVENT:
        return ConnectorType::SLOT_T;
    default:
        throw std::logic_error("unknown connector type");
    }
}

}
}
