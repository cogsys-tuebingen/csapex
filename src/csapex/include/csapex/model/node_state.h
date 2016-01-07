#ifndef NODE_STATE_H
#define NODE_STATE_H

/// COMPNENT
#include <csapex/model/memento.h>
#include <csapex/data/point.h>
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <csapex/utility/slim_signal.h>

namespace csapex
{

class NodeState : public Memento
{
public:
    typedef std::shared_ptr<NodeState> Ptr;
    typedef csapex::slim_signal::Signal<void()> SignalImpl;
    typedef std::shared_ptr< SignalImpl > Signal;

    NodeState(const NodeHandle *parent);
    ~NodeState();

    NodeState& operator = (const NodeState& rhs);

    virtual void writeYaml(YAML::Node& out) const override;
    virtual void readYaml(const YAML::Node& node) override;

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

    const NodeHandle* getParent() const;
    void setParent(const NodeHandle *value);
    Signal parent_changed;

    Memento::Ptr getParameterState() const;
    void setParameterState(const Memento::Ptr &value);

private:
    const NodeHandle* parent_;

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
