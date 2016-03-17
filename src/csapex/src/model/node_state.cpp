/// HEADER
#include <csapex/model/node_state.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/utility/yaml_io.hpp>

/// SYSTEM
#include <iostream>

using namespace csapex;

NodeState::NodeState(const NodeHandle *parent)
    : pos_changed(new SignalImpl),
      z_changed(new SignalImpl),
      color_changed(new SignalImpl),
      label_changed(new SignalImpl),
      minimized_changed(new SignalImpl),
      enabled_changed(new SignalImpl),
      flipped_changed(new SignalImpl),
      thread_changed(new SignalImpl),
      parent_changed(new SignalImpl),
      parent_(parent),

      z_(0), minimized_(false), enabled_(true), flipped_(false), thread_id_(-1),
      r_(-1), g_(-1), b_(-1)
{
    if(parent) {
        label_ = parent->getUUID().getFullName();
    }
}

NodeState::~NodeState()
{
}

NodeState& NodeState::operator = (const NodeState& rhs)
{
    setPos(rhs.pos_);
    setEnabled(rhs.enabled_);
    setZ(rhs.z_);
    setColor(rhs.r_, rhs.g_, rhs.b_);
    setMinimized(rhs.minimized_);
    setFlipped(rhs.flipped_);
    setLabel(rhs.label_);
    setThread(rhs.thread_name_, rhs.thread_id_);

    return *this;
}
Point NodeState::getPos() const
{
    return pos_;
}

void NodeState::setPos(const Point &value)
{
    if(pos_ != value) {
        pos_ = value;
        (*pos_changed)();
    }
}


long NodeState::getZ() const
{
    return z_;
}

void NodeState::setZ(long value)
{
    if(z_ != value) {
        z_ = value;
        (*z_changed)();
    }
}

void NodeState::getColor(int& r, int& g, int &b) const
{
    r = r_;
    g = g_;
    b = b_;
}

void NodeState::setColor(int r, int g, int b)
{
    if(r != r_ || b != b_ || g != g_) {
        r_ = r;
        g_ = g;
        b_ = b;
        (*color_changed)();
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
    return minimized_;
}

void NodeState::setMinimized(bool value)
{
    if(minimized_ != value) {
        minimized_ = value;
        (*minimized_changed)();
    }
}
bool NodeState::isEnabled() const
{
    return enabled_;
}

void NodeState::setEnabled(bool value)
{
    if(enabled_ != value) {
        enabled_ = value;
        (*enabled_changed)();
    }
}

bool NodeState::isFlipped() const
{
    return flipped_;
}

void NodeState::setFlipped(bool value)
{
    if(flipped_ != value) {
        flipped_ = value;
        (*flipped_changed)();
    }
}
Memento::Ptr NodeState::getParameterState() const
{
    return child_state_;
}

void NodeState::setParameterState(const Memento::Ptr &value)
{
    child_state_ = value;
}

const NodeHandle *NodeState::getParent() const
{
    return parent_;
}

void NodeState::setParent(const NodeHandle *value)
{
    if(parent_ != value) {
        parent_ = value;
        (*parent_changed)();
    }
}

int NodeState::getThreadId() const
{
    return thread_id_;
}

std::string NodeState::getThreadName() const
{
    return thread_name_;
}

void NodeState::setThread(const std::string& name, int id)
{
    if(thread_id_ != id || name != thread_name_) {
        thread_id_ = id;
        thread_name_ = name;

        (*thread_changed)();
    }
}


void NodeState::writeYaml(YAML::Node &out) const
{
    if(parent_) {
        out["type"] = parent_->getType();
        out["uuid"] = parent_->getUUID().getFullName();
    }
    out["label"] = label_;
    out["pos"][0] = pos_.x;
    out["pos"][1] = pos_.y;
    out["color"][0] = r_;
    out["color"][1] = g_;
    out["color"][2] = b_;
    out["z"] = z_;
    out["minimized"] = minimized_;
    out["enabled"] = enabled_;
    out["flipped"] = flipped_;

    try {
        if(parent_) {
            auto node = parent_->getNode().lock();
            if(node) {
                child_state_ = node->getParameterStateClone();
            }
        }
    } catch(const std::exception& e) {
        std::cerr << "cannot clone child state for node " << parent_->getUUID() << ": " << e.what() << std::endl;
        throw e;
    }

    if(child_state_.get()) {
        try {
            YAML::Node sub_node;
            child_state_->writeYaml(sub_node);
            out["state"] = sub_node;
        } catch(const std::exception& e) {
            std::cerr << "cannot save child state for node " << parent_->getUUID() << ": " << e.what() << std::endl;
            throw e;
        }
    }
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
            setLabel(parent_->getUUID().getFullName());
        }
    }

    if(node["pos"].IsDefined()) {
        double x = node["pos"][0].as<double>();
        double y = node["pos"][1].as<double>();
        Point p(x,y);
        setPos(p);
    }
    if(node["color"].IsDefined()) {
        int r = node["color"][0].as<int>();
        int g = node["color"][1].as<int>();
        int b = node["color"][2].as<int>();
        setColor(r, g, b);
    }
    if(node["z"].IsDefined()) {
        setZ(node["z"].as<long>());
    }

    if(node["state"].IsDefined()) {
        const YAML::Node& state_map = node["state"];
        auto node = parent_->getNode().lock();
        if(!node) {
            return;
        }
        child_state_ = node->getParameterStateClone();

        if(child_state_) {
            child_state_->readYaml(state_map);
        }
    }
}
