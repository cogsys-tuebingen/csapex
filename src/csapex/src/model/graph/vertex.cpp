/// HEADER
#include <csapex/model/graph/vertex.h>

/// PROJECT
#include <csapex/model/node_handle.h>

using namespace csapex;
using namespace csapex::graph;

Vertex::Vertex(NodeHandlePtr node)
    : node_(node)
{

}

NodeHandlePtr Vertex::getNodeHandle() const
{
    return node_;
}

NodeCharacteristics& Vertex::getNodeCharacteristics() const
{
    return characteristics_;
}
