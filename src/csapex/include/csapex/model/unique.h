#ifndef UNIQUE_H
#define UNIQUE_H

/// COMPONENT
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <string>

namespace csapex
{
class Unique
{
public:
    Unique(const UUID &uuid);
    virtual ~Unique();

    UUID getUUID() const;

protected:
    void setUUID(const UUID &uuid);

private:
    UUID uuid_;
};
}

#endif // UNIQUE_H