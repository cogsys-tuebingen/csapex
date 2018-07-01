#ifndef NODE_CHARACTERISTICS_H
#define NODE_CHARACTERISTICS_H

/// PROJECT
#include <csapex/serialization/serializable.h>

namespace csapex
{
class NodeCharacteristics : public Serializable
{
protected:
    CLONABLE_IMPLEMENTATION(NodeCharacteristics);

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

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;
};

}  // namespace csapex

#endif  // NODE_CHARACTERISTICS_H
