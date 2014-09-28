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

    NodeState(const Node* parent);

    void copyFrom (const Ptr &rhs);

    virtual void writeYaml(YAML::Node& out) const;
    virtual void readYaml(const YAML::Node& node);

public:
    QPoint getPos() const;
    void setPos(const QPoint &value);

    std::string getLabel() const;
    void setLabel(const std::string &label);

    bool isMinimized() const;
    void setMinimized(bool value);

    bool isEnabled() const;
    void setEnabled(bool value);

    bool isFlipped() const;
    void setFlipped(bool value);

    Memento::Ptr getParameterState() const;
    void setParameterState(const Memento::Ptr &value);

    const Node* getParent() const;
    void setParent(Node *value);

    int getThread() const;
    void setThread(int id);

private:
    const Node* parent;

    mutable Memento::Ptr child_state;

    std::string label_;
    QPoint pos;

    bool minimized;
    bool enabled;
    bool flipped;

    int thread;
};

}

#endif // NODE_STATE_H
