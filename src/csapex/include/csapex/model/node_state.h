#ifndef NODE_STATE_H
#define NODE_STATE_H

/// COMPNENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/memento.h>

/// SYSTEM
#include <QPoint>

namespace csapex
{

struct NodeState : public Memento {
    typedef boost::shared_ptr<NodeState> Ptr;

    NodeState(Node* parent);

    void copyFrom (const Ptr &rhs);

    virtual void writeYaml(YAML::Emitter& out) const;
    virtual void readYaml(const YAML::Node& node);

    Node* parent;

    mutable Memento::Ptr boxed_state;

    std::string label_;
    QPoint pos;

    bool minimized;
    bool enabled;
};

}

#endif // NODE_STATE_H
