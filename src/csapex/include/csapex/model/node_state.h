#ifndef NODE_STATE_H
#define NODE_STATE_H

/// COMPNENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/memento.h>

/// SYSTEM
#include <QPoint>
#include <boost/signals2/signal.hpp>

namespace csapex
{

struct NodeState : public Memento {
    typedef std::shared_ptr<NodeState> Ptr;
    typedef boost::signals2::signal<void()> SignalImpl;
    typedef std::shared_ptr< SignalImpl > Signal;

    NodeState(const NodeWorker *parent);

    NodeState& operator = (const NodeState& rhs);

    virtual void writeYaml(YAML::Node& out) const;
    virtual void readYaml(const YAML::Node& node);

public:
    QPoint getPos() const;
    void setPos(const QPoint &value);
    Signal pos_changed;

    std::string getLabel() const;
    void setLabel(const std::string &label);
    Signal label_changed;

    bool isMinimized() const;
    void setMinimized(bool value);
    Signal minimized_changed;

    bool isEnabled() const;
    void setEnabled(bool value);
    Signal enabled_changed;

    bool isFlipped() const;
    void setFlipped(bool value);
    Signal flipped_changed;

    int getThread() const;
    void setThread(int id);
    Signal thread_changed;

    const NodeWorker* getParent() const;
    void setParent(const NodeWorker *value);
    Signal parent_changed;

    Memento::Ptr getParameterState() const;
    void setParameterState(const Memento::Ptr &value);

private:
    const NodeWorker* parent;

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
