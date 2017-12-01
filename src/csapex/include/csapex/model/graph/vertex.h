#ifndef VERTEX_H_
#define VERTEX_H_

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/model/node_characteristics.h>
#include <csapex/utility/uuid.h>

/// SYTEM
#include <vector>

namespace csapex
{

namespace graph
{

class Vertex
{
public:
    Vertex(NodeFacadePtr node);
    ~Vertex();

    UUID getUUID() const;
    AUUID getAUUID() const;

    NodeFacadePtr getNodeFacade() const;
    NodeCharacteristics& getNodeCharacteristics() const;

    std::vector<VertexPtr> getParents() const;
    void addParent(VertexPtr parent);
    void removeParent(Vertex *parent);

    std::vector<VertexPtr> getChildren() const;
    void addChild(VertexPtr child);
    void removeChild(Vertex *child);

    void detach();

private:
    NodeFacadePtr node_;

    mutable NodeCharacteristics characteristics_;

    std::vector<VertexWeakPtr> children_;
    std::vector<VertexWeakPtr> parents_;
};

}

}

#endif
