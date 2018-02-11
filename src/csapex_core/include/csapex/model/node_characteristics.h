#ifndef NODE_CHARACTERISTICS_H
#define NODE_CHARACTERISTICS_H

/// PROJECT
#include <csapex/serialization/serializable.h>

namespace csapex
{

class NodeCharacteristics : public Serializable
{
public:
    NodeCharacteristics();

public:
    int depth;
    int component;

    bool is_joining_vertex;
    bool is_joining_vertex_counterpart;

    bool is_combined_by_joining_vertex;
    bool is_leading_to_joining_vertex;

    bool is_leading_to_essential_vertex;

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(const SerializationBuffer& data) override;

protected:
    virtual std::shared_ptr<Clonable> makeEmptyClone() const override;
};

}

#endif // NODE_CHARACTERISTICS_H
