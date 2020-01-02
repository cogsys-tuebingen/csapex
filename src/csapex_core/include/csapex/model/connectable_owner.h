#ifndef CONNECTABLE_OWNER_H
#define CONNECTABLE_OWNER_H

/// PROJECT
#include <csapex/model/unique.h>

namespace csapex
{
class ConnectableOwner : public Unique
{
protected:
    ConnectableOwner(const UUID& uuid);
    ~ConnectableOwner() override;
};
}  // namespace csapex

#endif  // CONNECTABLE_OWNER_H
