/// HEADER
#include <csapex/model/node_state.h>

/// COMPONENT
#include <csapex/model/node.h>

using namespace csapex;

NodeState::NodeState(Node* parent)
    : parent(parent), minimized(false), enabled(true), flipped(false)
{
    if(parent) {
        label_ = parent->getUUID().getFullName();
    }
}

void NodeState::copyFrom(const NodeState::Ptr& rhs)
{
    operator =(*rhs);
    child_state = parent->getState();
    if(rhs->child_state) {
        *child_state = *rhs->child_state;
    }
}

void NodeState::readYaml(const YAML::Node &node)
{
    if(node.FindValue("minimized")) {
        node["minimized"] >> minimized;
    }

    if(node.FindValue("enabled")) {
        node["enabled"] >> enabled;
    }

    if(node.FindValue("flipped")) {
        node["flipped"] >> flipped;
    }

    if(node.FindValue("label")) {
        node["label"] >> label_;
        if(label_.empty()) {
            label_ = parent->getUUID();
        }
    }

    if(node.FindValue("pos")) {
        double x, y;
        node["pos"][0] >> x;
        node["pos"][1] >> y;
        pos.setX(x);
        pos.setY(y);
    }

    if(node.FindValue("state")) {
        const YAML::Node& state_map = node["state"];
        child_state = parent->getState();

        if(child_state) {
            child_state->readYaml(state_map);
        }
    }
}


void NodeState::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Flow;
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
        child_state = parent->getState();
    }

    if(child_state.get()) {
        out << YAML::Key << "state";
        out << YAML::Value << YAML::BeginMap;
        child_state->writeYaml(out);
        out << YAML::EndMap;
    }

    out << YAML::EndMap;
}

