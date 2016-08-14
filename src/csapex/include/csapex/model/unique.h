#ifndef UNIQUE_H
#define UNIQUE_H

/// COMPONENT
#include <csapex/utility/uuid.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <string>

namespace csapex
{
class CSAPEX_EXPORT Unique
{
public:
    Unique(const UUID &uuid);
    virtual ~Unique();

    UUID getUUID() const;

protected:
    virtual void setUUID(const UUID &uuid);

private:
    UUID uuid_;
};
}

#endif // UNIQUE_H
