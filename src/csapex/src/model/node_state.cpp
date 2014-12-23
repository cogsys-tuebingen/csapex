/// HEADER
#include <csapex/model/node_state.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/utility/yaml_io.hpp>

using namespace csapex;

NodeState::NodeState(const NodeWorker *parent)
    : parent(parent), minimized(false), enabled(true), flipped(false), thread(-1)
{
    if(parent) {
        label_ = parent->getUUID().getFullName();
    }
}

void NodeState::readYaml(const YAML::Node &node)
{
    if(node["minimized"].IsDefined()) {
        minimized = node["minimized"].as<bool>();
    }

    if(node["enabled"].IsDefined()) {
        enabled = node["enabled"].as<bool>();
    }

    if(node["flipped"].IsDefined()) {
        flipped = node["flipped"].as<bool>();
    }

    if(node["label"].IsDefined()) {
        label_ = node["label"].as<std::string>();
        if(label_.empty()) {
            label_ = parent->getUUID();
        }
    }

    if(node["pos"].IsDefined()) {
        double x = node["pos"][0].as<double>();
        double y = node["pos"][1].as<double>();
        pos.setX(x);
        pos.setY(y);
    }

    if(node["thread"].IsDefined()) {
        thread = node["thread"].as<int>();
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
    pos = value;
}

std::string NodeState::getLabel() const
{
    return label_;
}

void NodeState::setLabel(const std::string &label)
{
    label_ = label;
}

bool NodeState::isMinimized() const
{
    return minimized;
}

void NodeState::setMinimized(bool value)
{
    minimized = value;
}
bool NodeState::isEnabled() const
{
    return enabled;
}

void NodeState::setEnabled(bool value)
{
    enabled = value;
}

bool NodeState::isFlipped() const
{
    return flipped;
}

void NodeState::setFlipped(bool value)
{
    flipped = value;
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

void NodeState::setParent(NodeWorker *value)
{
    parent = value;
}

int NodeState::getThread() const
{
    return thread;
}

void NodeState::setThread(int id)
{
    thread = id;
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
