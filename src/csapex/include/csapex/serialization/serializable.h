#ifndef SERIALIZABLE_H
#define SERIALIZABLE_H

/// PROJECT
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/model/clonable.h>

/// SYSTEM
#include <inttypes.h>
#include <vector>

namespace csapex
{

class Serializable : public Clonable
{
public:
    virtual ~Serializable();

    virtual void serialize(SerializationBuffer &data) const = 0;
    virtual void deserialize(SerializationBuffer& data) = 0;
};

}

#endif
