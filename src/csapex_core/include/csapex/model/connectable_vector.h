#ifndef PORT_VECTOR_H
#define PORT_VECTOR_H

/// MODEL
#include <csapex/model/connectable.h>
#include <csapex/model/connector_description.h>

/// SYSTEM
#include <memory>

namespace csapex
{
template <typename T>
class CSAPEX_CORE_EXPORT ConnectableVector : public std::vector<std::shared_ptr<T>>
{
public:
    std::vector<ConnectorDescription> getDescription() const;
};

}  // namespace csapex

#endif  // PORT_VECTOR_H
