#ifndef NODE_STATE_H
#define NODE_STATE_H

/// COMPNENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/memento.h>
#include <csapex/data/point.h>

/// SYSTEM
#include <boost/signals2/signal.hpp>

namespace csapex
{

class NodeState : public Memento
{
public:
    typedef std::shared_ptr<NodeState> Ptr;
    typedef boost::signals2::signal<void()> SignalImpl;
    typedef std::shared_ptr< SignalImpl > Signal;

    NodeState(const NodeWorker *parent);

    NodeState& operator = (const NodeState& rhs);

    virtual void writeYaml(YAML::Node& out) const;
    virtual void readYaml(const YAML::Node& node);

public:
    Point getPos() const;
    void setPos(const Point &value);
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

    int getThreadId() const;
    std::string getThreadName() const;
    void setThread(const std::string& name, int id);
    Signal thread_changed;

    const NodeWorker* getParent() const;
    void setParent(const NodeWorker *value);
    Signal parent_changed;

    Memento::Ptr getParameterState() const;
    void setParameterState(const Memento::Ptr &value);

private:
    const NodeWorker* parent_;

    mutable Memento::Ptr child_state_;

    std::string label_;
    Point pos_;

    bool minimized_;
    bool enabled_;
    bool flipped_;

    int thread_id_;
    std::string thread_name_;
};

}

#endif // NODE_STATE_H
