/// HEADER
#include <csapex/model/node_characteristics.h>

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>

using namespace csapex;

NodeCharacteristics::NodeCharacteristics()
    : depth(-1),
      component(-1),

      is_joining_vertex(false),
      is_joining_vertex_counterpart(false),
      is_combined_by_joining_vertex(false),
      is_leading_to_joining_vertex(false),

      is_leading_to_essential_vertex(false)
{

}


std::shared_ptr<Clonable> NodeCharacteristics::makeEmptyClone() const
{
    return std::make_shared<NodeCharacteristics>();
}

void NodeCharacteristics::serialize(SerializationBuffer &data) const
{
    data << depth;
    data << component;
    data << is_joining_vertex;
    data << is_joining_vertex_counterpart;
    data << is_combined_by_joining_vertex;
    data << is_leading_to_joining_vertex;
    data << is_leading_to_essential_vertex;
}
void NodeCharacteristics::deserialize(const SerializationBuffer& data)
{
    data >> depth;
    data >> component;
    data >> is_joining_vertex;
    data >> is_joining_vertex_counterpart;
    data >> is_combined_by_joining_vertex;
    data >> is_leading_to_joining_vertex;
    data >> is_leading_to_essential_vertex;
}
