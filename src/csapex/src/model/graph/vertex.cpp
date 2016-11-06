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

Vertex::~Vertex()
{
    detach();
}

void Vertex::detach()
{
    for(const VertexWeakPtr& w_parent : parents_) {
        if(VertexPtr parent = w_parent.lock()) {
            parent->removeChild(this);
        }
    }
    parents_.clear();

    for(const VertexWeakPtr& w_child : children_) {
        if(VertexPtr child = w_child.lock()) {
            child->removeParent(this);
        }
    }
    children_.clear();
}

NodeHandlePtr Vertex::getNodeHandle() const
{
    return node_;
}

NodeCharacteristics& Vertex::getNodeCharacteristics() const
{
    return characteristics_;
}


std::vector<VertexPtr> Vertex::getParents() const
{
    std::vector<VertexPtr> res;
    for(const VertexWeakPtr& parent : parents_) {
        res.push_back(parent.lock());
    }
    return res;
}

std::vector<VertexPtr> Vertex::getChildren() const
{
    std::vector<VertexPtr> res;
    for(const VertexWeakPtr& child : children_) {
        res.push_back(child.lock());
    }
    return res;
}

void Vertex::addParent(VertexPtr parent)
{
    parents_.push_back(parent);
}

void Vertex::removeParent(Vertex* parent)
{
    for(auto it = parents_.begin(); it != parents_.end();) {
        if(VertexPtr p = it->lock()) {
            if(p.get() == parent) {
                it = parents_.erase(it);
            } else {
                ++it;
            }
        } else {
            it = parents_.erase(it);
        }
    }
}

void Vertex::addChild(VertexPtr child)
{
    children_.push_back(child);
}

void Vertex::removeChild(Vertex* child)
{
    for(auto it = children_.begin(); it != children_.end();) {
        if(VertexPtr p = it->lock()) {
            if(p.get() == child) {
                it = children_.erase(it);
            } else {
                ++it;
            }
        } else {
            it = children_.erase(it);
        }
    }
}
