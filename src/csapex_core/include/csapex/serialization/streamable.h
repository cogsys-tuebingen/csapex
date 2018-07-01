#ifndef STREAMABLE_H
#define STREAMABLE_H

/// PROJECT
#include <csapex/serialization/serializable.h>

namespace csapex
{
class Streamable : public Serializable
{
public:
    virtual uint8_t getPacketType() const = 0;
};

}  // namespace csapex

#endif  // STREAMABLE_H
