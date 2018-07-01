#ifndef UNIQUE_H
#define UNIQUE_H

/// COMPONENT
#include <csapex/utility/uuid.h>
#include <csapex_core/csapex_core_export.h>

/// SYSTEM
#include <string>

namespace csapex
{
class CSAPEX_CORE_EXPORT Unique
{
public:
    Unique(const UUID& uuid);
    virtual ~Unique();

    UUID getUUID() const;
    AUUID getAUUID() const;

protected:
    virtual void setUUID(const UUID& uuid);

private:
    UUID uuid_;
};
}  // namespace csapex

#endif  // UNIQUE_H
