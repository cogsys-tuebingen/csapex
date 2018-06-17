#ifndef SERIALIZABLE_H
#define SERIALIZABLE_H

/// PROJECT
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/model/clonable.h>
#include <csapex/serialization/semantic_version.h>

/// SYSTEM
#include <inttypes.h>
#include <vector>

namespace csapex
{

class Serializable : public Clonable
{
public:
    virtual ~Serializable();

    virtual SemanticVersion getVersion() const;

    virtual void serialize(SerializationBuffer &data, SemanticVersion& version) const = 0;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) = 0;

    void serializeVersioned(SerializationBuffer &data) const;
    void deserializeVersioned(const SerializationBuffer& data);
};

}

#endif
