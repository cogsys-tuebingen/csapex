#ifndef UNIQUE_H
#define UNIQUE_H

/// SYSTEM
#include <string>

namespace csapex
{
class Unique
{
public:
    Unique(const std::string& uuid);
    virtual ~Unique();

    std::string UUID() const;

protected:
    void setUUID(const std::string& uuid);

private:
    std::string uuid_;
};
}

#endif // UNIQUE_H
