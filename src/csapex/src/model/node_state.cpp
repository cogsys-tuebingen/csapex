/// HEADER
#include <csapex/model/node_state.h>

/// COMPONENT
#include <csapex/model/node.h>

using namespace csapex;

//NodeState::NodeState()
//    : parent(NULL), minimized(false), enabled(true)
//{}
NodeState::NodeState(Node* parent)
    : parent(parent), minimized(false), enabled(true)
{}

void NodeState::copyFrom(const NodeState::Ptr& rhs)
{
    operator =(*rhs);
    boxed_state = parent->getState();
    if(rhs->boxed_state) {
        *boxed_state = *rhs->boxed_state;
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

    if(node.FindValue("label")) {
        node["label"] >> label_;
        if(label_.empty()) {
            label_ = parent->UUID();
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
        boxed_state = parent->getState();

        if(boxed_state) {
            boxed_state->readYaml(state_map);
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
        out << YAML::Value << parent->uuid_;
    }
    out << YAML::Key << "label";
    out << YAML::Value << label_;
    out << YAML::Key << "pos";
    out << YAML::Value << YAML::BeginSeq << pos.x() << pos.y() << YAML::EndSeq;
    out << YAML::Key << "minimized";
    out << YAML::Value << minimized;
    out << YAML::Key << "enabled";
    out << YAML::Value << enabled;

    if(parent) {
        boxed_state = parent->getState();
    }

    if(boxed_state.get()) {
        out << YAML::Key << "state";
        out << YAML::Value << YAML::BeginMap;
        boxed_state->writeYaml(out);
        out << YAML::EndMap;
    }

    out << YAML::EndMap;
}

