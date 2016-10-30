/// HEADER
#include <csapex/model/node_state.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex/model/generic_state.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

NodeState::NodeState(const NodeHandle *parent)
    : pos_changed(new SignalImpl),
      z_changed(new SignalImpl),
      color_changed(new SignalImpl),
      label_changed(new SignalImpl),
      minimized_changed(new SignalImpl),
      muted_changed(new SignalImpl),
      enabled_changed(new SignalImpl),
      active_changed(new SignalImpl),
      flipped_changed(new SignalImpl),
      thread_changed(new SignalImpl),
      execution_mode_changed(new SignalImpl),
      logger_level_changed(new SignalImpl),
      parent_changed(new SignalImpl),
      parent_(parent),

      z_(0), minimized_(false), muted_(false), enabled_(true), active_(false), flipped_(false),
      logger_level_(1), thread_id_(-1),
      r_(-1), g_(-1), b_(-1), exec_mode_(ExecutionMode::SEQUENTIAL)
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
    // first change all values
    pos_ = rhs.pos_;
    enabled_ = rhs.enabled_;
    active_ = rhs.active_;
    z_ = rhs.z_;
    r_ = rhs.r_;
    g_ = rhs.g_;
    b_ = rhs.b_;
    minimized_ = rhs.minimized_;
    muted_ = rhs.muted_;
    flipped_ = rhs.flipped_;
    label_ = rhs.label_;
    thread_name_ = rhs.thread_name_;
    thread_id_ = rhs.thread_id_;
    exec_mode_ = rhs.exec_mode_;
    logger_level_ = rhs.logger_level_;

    dictionary = rhs.dictionary;

    // then trigger the signals
    (*pos_changed)();
    (*enabled_changed)();
    (*active_changed)();
    (*z_changed)();
    (*color_changed)();
    (*minimized_changed)();
    (*muted_changed)();
    (*flipped_changed)();
    (*label_changed)();
    (*thread_changed)();
    (*execution_mode_changed)();
    (*logger_level_changed)();

    return *this;
}
Point NodeState::getPos() const
{
    return pos_;
}

void NodeState::setPos(const Point &value, bool quiet)
{
    if(pos_ != value) {
        pos_ = value;
        if(!quiet) {
            (*pos_changed)();
        }
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

bool NodeState::isMuted() const
{
    return muted_;
}

void NodeState::setMuted(bool value)
{
    if(muted_ != value) {
        muted_ = value;
        (*muted_changed)();
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

bool NodeState::isActive() const
{
    return active_;
}

void NodeState::setActive(bool value)
{
    if(active_ != value) {
        active_ = value;
        (*active_changed)();
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


ExecutionMode NodeState::getExecutionMode() const
{
    return exec_mode_;
}
void NodeState::setExecutionMode(ExecutionMode mode)
{
    if(exec_mode_ != mode) {
        exec_mode_ = mode;

        (*execution_mode_changed)();
    }
}

int NodeState::getLoggerLevel() const
{
    return logger_level_;
}
void NodeState::setLoggerLevel(int level)
{
    if(logger_level_ != level) {
        logger_level_ = level;

        (*logger_level_changed)();
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
    out["muted"] = muted_;
    out["enabled"] = enabled_;
    out["flipped"] = flipped_;
    out["exec_mode"] = (int) exec_mode_;
    out["logger_level"] = logger_level_;

    if(!dictionary.empty()) {
        YAML::Node dict(YAML::NodeType::Sequence);
        for(const auto& pair : dictionary) {
            YAML::Node n(YAML::NodeType::Map);

            n["key"] = pair.first;

            if(pair.second.type() == typeid(int)) {
                n["int"] = boost::any_cast<int> (pair.second);

            } else if(pair.second.type() == typeid(double)) {
                n["double"] = boost::any_cast<double> (pair.second);

            } else if(pair.second.type() == typeid(bool)) {
                n["bool"] = boost::any_cast<bool> (pair.second);

            } else if(pair.second.type() == typeid(std::string)) {
                n["string"] = boost::any_cast<std::string> (pair.second);

            } else if(pair.second.type() == typeid(std::vector<std::string>)) {
                n["stringv"] = boost::any_cast<std::vector<std::string>> (pair.second);
            }

            dict.push_back(n);
        }
        out["dict"] = dict;
    }

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

    if(node["muted"].IsDefined()) {
        setMuted(node["muted"].as<bool>());
    }

    if(node["enabled"].IsDefined()) {
        setEnabled(node["enabled"].as<bool>());
    }

    if(node["flipped"].IsDefined()) {
        setFlipped(node["flipped"].as<bool>());
    }

    if(node["exec_mode"].IsDefined()) {
        setExecutionMode(static_cast<ExecutionMode>(node["exec_mode"].as<int>()));
    }

    if(node["label"].IsDefined()) {
        setLabel(node["label"].as<std::string>());
        if(label_.empty()) {
            setLabel(parent_->getUUID().getFullName());
        }
    }

    if(node["logger_level"].IsDefined()) {
        setLoggerLevel(node["logger_level"].as<int>());
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

    if(node["dict"].IsDefined()) {
        const YAML::Node& dict = node["dict"];
        if(dict.Type() == YAML::NodeType::Sequence) {

            for(auto it = dict.begin(); it != dict.end(); ++it) {
                YAML::Node entry = *it;
                std::string key = entry["key"].as<std::string>();

                if(entry["int"].IsDefined()) {
                    dictionary[key] = entry["int"].as<int>();

                } else if(entry["double"].IsDefined()) {
                    dictionary[key] = entry["double"].as<double>();

                } else if(entry["bool"].IsDefined()) {
                    dictionary[key] = entry["bool"].as<bool>();

                } else if(entry["string"].IsDefined()) {
                    dictionary[key] = entry["string"].as<std::string>();

                } else if(entry["stringv"].IsDefined()) {
                    dictionary[key] = entry["stringv"].as<std::vector<std::string>>();
                }
            }
        }
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

bool NodeState::hasDictionaryEntry(const std::string& key) const
{
    return dictionary.find(key) != dictionary.end();
}

void NodeState::deleteDictionaryEntry(const std::string &key)
{
    dictionary.erase(key);
}

template <typename T>
T NodeState::getDictionaryEntry(const std::string& key) const
{
    return boost::any_cast<T>(dictionary.at(key));
}
template <typename T>
void NodeState::setDictionaryEntry(const std::string& key, const T& value)
{
    dictionary[key] = value;
}

namespace csapex
{
template void NodeState::setDictionaryEntry<int>(const std::string&, const int&);
template void NodeState::setDictionaryEntry<double>(const std::string&, const double&);
template void NodeState::setDictionaryEntry<bool>(const std::string&, const bool&);
template void NodeState::setDictionaryEntry<std::string>(const std::string&, const std::string&);
template void NodeState::setDictionaryEntry<std::vector<std::string>>(const std::string&, const std::vector<std::string>&);

template int NodeState::getDictionaryEntry<int>(const std::string&) const;
template double NodeState::getDictionaryEntry<double>(const std::string&) const;
template bool NodeState::getDictionaryEntry<bool>(const std::string&) const;
template std::string NodeState::getDictionaryEntry<std::string>(const std::string&) const;
template std::vector<std::string> NodeState::getDictionaryEntry<std::vector<std::string>>(const std::string&) const;
}
