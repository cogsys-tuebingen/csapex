/// HEADER
#include <csapex/model/node_state.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/utility/yaml_io.hpp>

using namespace csapex;

NodeState::NodeState(const NodeWorker *parent)
    : pos_changed(new SignalImpl),
      label_changed(new SignalImpl),
      minimized_changed(new SignalImpl),
      enabled_changed(new SignalImpl),
      flipped_changed(new SignalImpl),
      thread_changed(new SignalImpl),
      parent_changed(new SignalImpl),
      parent(parent), minimized(false), enabled(true), flipped(false), thread(-1)
{
    if(parent) {
        label_ = parent->getUUID().getFullName();
    }
}

NodeState& NodeState::operator = (const NodeState& rhs)
{
    setPos(rhs.pos);
    setEnabled(rhs.enabled);
    setMinimized(rhs.minimized);
    setFlipped(rhs.flipped);
    setParent(rhs.parent);
    setLabel(rhs.label_);
    setThread(rhs.thread);

    return *this;
}

void NodeState::readYaml(const YAML::Node &node)
{
    if(node["minimized"].IsDefined()) {
        setMinimized(node["minimized"].as<bool>());
    }

    if(node["enabled"].IsDefined()) {
        setEnabled(node["enabled"].as<bool>());
    }

    if(node["flipped"].IsDefined()) {
        setFlipped(node["flipped"].as<bool>());
    }

    if(node["label"].IsDefined()) {
        setLabel(node["label"].as<std::string>());
        if(label_.empty()) {
            setLabel(parent->getUUID());
        }
    }

    if(node["pos"].IsDefined()) {
        double x = node["pos"][0].as<double>();
        double y = node["pos"][1].as<double>();
        QPoint p(x,y);
        setPos(p);
    }

    if(node["thread"].IsDefined()) {
        setThread(node["thread"].as<int>());
    }

    if(node["state"].IsDefined()) {
        const YAML::Node& state_map = node["state"];
        child_state = parent->getNode()->getParameterStateClone();

        if(child_state) {
            child_state->readYaml(state_map);
        }
    }
}
QPoint NodeState::getPos() const
{
    return pos;
}

void NodeState::setPos(const QPoint &value)
{
    if(pos != value) {
        pos = value;
        (*pos_changed)();
    }
}

std::string NodeState::getLabel() const
{
    return label_;
}

void NodeState::setLabel(const std::string &label)
{
    if(label_ != label) {
        label_ = label;
        (*label_changed)();
    }
}

bool NodeState::isMinimized() const
{
    return minimized;
}

void NodeState::setMinimized(bool value)
{
    if(minimized != value) {
        minimized = value;
        (*minimized_changed)();
    }
}
bool NodeState::isEnabled() const
{
    return enabled;
}

void NodeState::setEnabled(bool value)
{
    if(enabled != value) {
        enabled = value;
        (*enabled_changed)();
    }
}

bool NodeState::isFlipped() const
{
    return flipped;
}

void NodeState::setFlipped(bool value)
{
    if(flipped != value) {
        flipped = value;
        (*flipped_changed)();
    }
}
Memento::Ptr NodeState::getParameterState() const
{
    return child_state;
}

void NodeState::setParameterState(const Memento::Ptr &value)
{
    child_state = value;
}

const NodeWorker *NodeState::getParent() const
{
    return parent;
}

void NodeState::setParent(const NodeWorker *value)
{
    if(parent != value) {
        parent = value;
        (*parent_changed)();
    }
}

int NodeState::getThread() const
{
    return thread;
}

void NodeState::setThread(int id)
{
    if(thread != id) {
        thread = id;
        (*thread_changed)();
    }
}


void NodeState::writeYaml(YAML::Node &out) const
{
    if(parent) {
        out["type"] = parent->getType();
        out["uuid"] = parent->getUUID();
    }
    out["label"] = label_;
    out["pos"][0] = pos.x();
    out["pos"][1] = pos.y();
    out["minimized"] = minimized;
    out["enabled"] = enabled;
    out["flipped"] = flipped;
    out["thread"] = thread;

    try {
        if(parent) {
            child_state = parent->getNode()->getParameterStateClone();
        }
    } catch(const std::exception& e) {
        std::cerr << "cannot clone child state for node " << parent->getUUID() << ": " << e.what() << std::endl;
        throw e;
    }

    if(child_state.get()) {
        try {
            YAML::Node sub_node;
            child_state->writeYaml(sub_node);
            out["state"] = sub_node;
        } catch(const std::exception& e) {
            std::cerr << "cannot save child state for node " << parent->getUUID() << ": " << e.what() << std::endl;
            throw e;
        }
    }
}
