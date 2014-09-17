/// HEADER
#include <csapex/model/node_state.h>

/// COMPONENT
#include <csapex/model/node.h>

using namespace csapex;

NodeState::NodeState(const Node *parent)
    : parent(parent), minimized(false), enabled(true), flipped(false)
{
    if(parent) {
        label_ = parent->getUUID().getFullName();
    }
}

void NodeState::copyFrom(const NodeState::Ptr& rhs)
{
    operator =(*rhs);
    child_state = parent->getParameterState();
    if(rhs->child_state) {
        *child_state = *rhs->child_state;
    }
}

void NodeState::readYaml(const YAML::Node &node)
{
    if(exists(node, "minimized")) {
        node["minimized"] >> minimized;
    }

    if(exists(node, "enabled")) {
        node["enabled"] >> enabled;
    }

    if(exists(node, "flipped")) {
        node["flipped"] >> flipped;
    }

    if(exists(node, "label")) {
        node["label"] >> label_;
        if(label_.empty()) {
            label_ = parent->getUUID();
        }
    }

    if(exists(node, "pos")) {
        double x, y;
        node["pos"][0] >> x;
        node["pos"][1] >> y;
        pos.setX(x);
        pos.setY(y);
    }

    if(exists(node, "state")) {
        const YAML::Node& state_map = node["state"];
        child_state = parent->getParameterState();

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

const Node *NodeState::getParent() const
{
    return parent;
}

void NodeState::setParent(Node *value)
{
    parent = value;
}


void NodeState::writeYaml(YAML::Emitter &out) const
{
    out << YAML::BeginMap;
    if(parent) {
        out << YAML::Key << "type";
        out << YAML::Value << parent->getType();
        out << YAML::Key << "uuid";
        out << YAML::Value << parent->getUUID();
    }
    out << YAML::Key << "label";
    out << YAML::Value << label_;
    out << YAML::Key << "pos";
    out << YAML::Value << YAML::BeginSeq << pos.x() << pos.y() << YAML::EndSeq;
    out << YAML::Key << "minimized";
    out << YAML::Value << minimized;
    out << YAML::Key << "enabled";
    out << YAML::Value << enabled;
    out << YAML::Key << "flipped";
    out << YAML::Value << flipped;

    if(parent) {
        child_state = parent->getParameterState();
    }

    if(child_state.get()) {
        out << YAML::Key << "state";
        out << YAML::Value << YAML::BeginMap;
        child_state->writeYaml(out);
        out << YAML::EndMap;
    }

    out << YAML::EndMap;
}
