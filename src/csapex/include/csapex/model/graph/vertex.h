#ifndef VERTEX_H_
#define VERTEX_H_

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/model/node_characteristics.h>

namespace csapex
{

namespace graph
{

class Vertex
{
public:
    Vertex(NodeHandlePtr node);

    NodeHandlePtr getNodeHandle() const;
    NodeCharacteristics& getNodeCharacteristics() const;

private:
    NodeHandlePtr node_;

    mutable NodeCharacteristics characteristics_;
};

}

}

#endif
