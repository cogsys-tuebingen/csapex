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
    Unique(const std::string& uuid);
    virtual ~Unique();

    UUID getUUID() const;

protected:
    void setUUID(const std::string& uuid);

private:
    UUID uuid_;
};
}

#endif // UNIQUE_H
