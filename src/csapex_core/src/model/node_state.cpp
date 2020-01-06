/// HEADER
#include <csapex/model/node_state.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex/model/generic_state.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

NodeState::NodeState(const NodeHandle* parent)
  : parent_(parent)
  ,

  max_frequency_(0.0)
  , z_(0)
  , minimized_(false)
  , muted_(false)
  , enabled_(true)
  , active_(false)
  , flipped_(false)
  , logger_level_(1)
  , thread_id_(-1)
  , r_(-1)
  , g_(-1)
  , b_(-1)
  , exec_mode_(ExecutionMode::SEQUENTIAL)
  , exec_type_(ExecutionType::AUTO)
{
    if (parent) {
        label_ = parent->getUUID().getFullName();
        if (NodePtr node = parent->getNode().lock()) {
            parameter_state = node->getParameterState();
        } else {
            apex_fail("cannot get node in node state");
        }

    } else {
        parameter_state = std::make_shared<GenericState>();
    }
}

NodeState::NodeState() : NodeState(nullptr)
{
}

NodeState::~NodeState()
{
}

NodeState& NodeState::operator=(const NodeState& rhs)
{
    // first change all values
    max_frequency_ = rhs.max_frequency_;
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
    exec_type_ = rhs.exec_type_;
    logger_level_ = rhs.logger_level_;

    dictionary = rhs.dictionary;

    parameter_state->setFrom(*rhs.parameter_state);

    // then trigger the signals
    (max_frequency_changed)();
    (pos_changed)();
    (enabled_changed)();
    (active_changed)();
    (z_changed)();
    (color_changed)();
    (minimized_changed)();
    (muted_changed)();
    (flipped_changed)();
    (label_changed)();
    (thread_changed)();
    (execution_mode_changed)();
    (execution_type_changed)();
    (logger_level_changed)();

    return *this;
}

void NodeState::setMaximumFrequency(double f)
{
    if (max_frequency_ != f) {
        max_frequency_ = f;
        (max_frequency_changed)();
    }
}

double NodeState::getMaximumFrequency() const
{
    return max_frequency_;
}

Point NodeState::getPos() const
{
    return pos_;
}

void NodeState::setPos(const Point& value, bool quiet)
{
    if (pos_ != value) {
        pos_ = value;
        if (!quiet) {
            (pos_changed)();
        }
    }
}

long NodeState::getZ() const
{
    return z_;
}

void NodeState::setZ(long value)
{
    if (z_ != value) {
        z_ = value;
        (z_changed)();
    }
}

void NodeState::getColor(int& r, int& g, int& b) const
{
    r = r_;
    g = g_;
    b = b_;
}

void NodeState::setColor(int r, int g, int b)
{
    if (r != r_ || b != b_ || g != g_) {
        r_ = r;
        g_ = g;
        b_ = b;
        (color_changed)();
    }
}

std::string NodeState::getLabel() const
{
    return label_;
}

void NodeState::setLabel(const std::string& label)
{
    if (label_ != label) {
        label_ = label;
        (label_changed)();
    }
}

bool NodeState::isMinimized() const
{
    return minimized_;
}

void NodeState::setMinimized(bool value)
{
    if (minimized_ != value) {
        minimized_ = value;
        (minimized_changed)();
    }
}

bool NodeState::isMuted() const
{
    return muted_;
}

void NodeState::setMuted(bool value)
{
    if (muted_ != value) {
        muted_ = value;
        (muted_changed)();
    }
}
bool NodeState::isEnabled() const
{
    return enabled_;
}

void NodeState::setEnabled(bool value)
{
    if (enabled_ != value) {
        enabled_ = value;
        (enabled_changed)();
    }
}

bool NodeState::isActive() const
{
    return active_;
}

void NodeState::setActive(bool value)
{
    if (active_ != value) {
        active_ = value;
        (active_changed)();
    }
}

bool NodeState::isFlipped() const
{
    return flipped_;
}

void NodeState::setFlipped(bool value)
{
    if (flipped_ != value) {
        flipped_ = value;
        (flipped_changed)();
    }
}
GenericStatePtr NodeState::getParameterState() const
{
    return parameter_state;
}

void NodeState::setParameterState(const GenericStatePtr& value)
{
    parameter_state->setFrom(*value);
}

const NodeHandle* NodeState::getParent() const
{
    return parent_;
}

void NodeState::setParent(const NodeHandle* value)
{
    if (parent_ != value) {
        parent_ = value;
        (parent_changed)();
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
    if (thread_id_ != id || name != thread_name_) {
        thread_id_ = id;
        thread_name_ = name;

        (thread_changed)();
    }
}

ExecutionMode NodeState::getExecutionMode() const
{
    return exec_mode_;
}
void NodeState::setExecutionMode(ExecutionMode mode)
{
    if (exec_mode_ != mode) {
        exec_mode_ = mode;

        (execution_mode_changed)();
    }
}

ExecutionType NodeState::getExecutionType() const
{
    return exec_type_;
}
void NodeState::setExecutionType(ExecutionType type)
{
    if (exec_type_ != type) {
        exec_type_ = type;

        (execution_type_changed)();
    }
}

int NodeState::getLoggerLevel() const
{
    return logger_level_;
}
void NodeState::setLoggerLevel(int level)
{
    if (logger_level_ != level) {
        logger_level_ = level;

        (logger_level_changed)();
    }
}

void NodeState::writeYaml(YAML::Node& out) const
{
    if (parent_) {
        out["type"] = parent_->getType();
        out["uuid"] = parent_->getUUID().getFullName();
    }
    out["max_frequency"] = max_frequency_;
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
    out["exec_mode"] = (int)exec_mode_;
    out["exec_type"] = (int)exec_type_;
    out["logger_level"] = logger_level_;

    if (!dictionary.empty()) {
        YAML::Node dict(YAML::NodeType::Sequence);
        for (const auto& pair : dictionary) {
            YAML::Node n(YAML::NodeType::Map);

            n["key"] = pair.first;

            if (pair.second.type() == typeid(int)) {
                n["int"] = std::any_cast<int>(pair.second);

            } else if (pair.second.type() == typeid(double)) {
                n["double"] = std::any_cast<double>(pair.second);

            } else if (pair.second.type() == typeid(bool)) {
                n["bool"] = std::any_cast<bool>(pair.second);

            } else if (pair.second.type() == typeid(std::string)) {
                n["string"] = std::any_cast<std::string>(pair.second);

            } else if (pair.second.type() == typeid(std::vector<std::string>)) {
                n["stringv"] = std::any_cast<std::vector<std::string>>(pair.second);
            }

            dict.push_back(n);
        }
        out["dict"] = dict;
    }

    try {
        YAML::Node sub_node;
        parameter_state->writeYaml(sub_node);
        out["state"] = sub_node;
    } catch (const std::exception& e) {
        std::cerr << "cannot save child state for node " << parent_->getUUID() << ": " << e.what() << std::endl;
        throw e;
    }
}

void NodeState::readYaml(const YAML::Node& node)
{
    if (node["max_frequency"].IsDefined()) {
        setMaximumFrequency(node["max_frequency"].as<double>());
    }

    if (node["minimized"].IsDefined()) {
        setMinimized(node["minimized"].as<bool>());
    }

    if (node["muted"].IsDefined()) {
        setMuted(node["muted"].as<bool>());
    }

    if (node["enabled"].IsDefined()) {
        setEnabled(node["enabled"].as<bool>());
    }

    if (node["flipped"].IsDefined()) {
        setFlipped(node["flipped"].as<bool>());
    }

    if (node["exec_mode"].IsDefined()) {
        setExecutionMode(static_cast<ExecutionMode>(node["exec_mode"].as<int>()));
    }
    if (node["exec_type"].IsDefined()) {
        setExecutionType(static_cast<ExecutionType>(node["exec_type"].as<int>()));
    }

    if (node["label"].IsDefined()) {
        setLabel(node["label"].as<std::string>());
        if (label_.empty()) {
            setLabel(parent_->getUUID().getFullName());
        }
    }

    if (node["logger_level"].IsDefined()) {
        setLoggerLevel(node["logger_level"].as<int>());
    }

    if (node["pos"].IsDefined()) {
        double x = node["pos"][0].as<double>();
        double y = node["pos"][1].as<double>();
        Point p(x, y);
        setPos(p);
    }
    if (node["color"].IsDefined()) {
        int r = node["color"][0].as<int>();
        int g = node["color"][1].as<int>();
        int b = node["color"][2].as<int>();
        setColor(r, g, b);
    }
    if (node["z"].IsDefined()) {
        setZ(node["z"].as<long>());
    }

    if (node["dict"].IsDefined()) {
        const YAML::Node& dict = node["dict"];
        if (dict.Type() == YAML::NodeType::Sequence) {
            for (auto it = dict.begin(); it != dict.end(); ++it) {
                YAML::Node entry = *it;
                std::string key = entry["key"].as<std::string>();

                if (entry["int"].IsDefined()) {
                    dictionary[key] = entry["int"].as<int>();

                } else if (entry["double"].IsDefined()) {
                    dictionary[key] = entry["double"].as<double>();

                } else if (entry["bool"].IsDefined()) {
                    dictionary[key] = entry["bool"].as<bool>();

                } else if (entry["string"].IsDefined()) {
                    dictionary[key] = entry["string"].as<std::string>();

                } else if (entry["stringv"].IsDefined()) {
                    dictionary[key] = entry["stringv"].as<std::vector<std::string>>();
                }
            }
        }
    }

    if (node["state"].IsDefined()) {
        const YAML::Node& state_map = node["state"];
        auto node = parent_->getNode().lock();
        if (!node) {
            return;
        }

        parameter_state->readYaml(state_map);
    }
}

void NodeState::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    data << max_frequency_;

    data << label_;
    data << pos_.x << pos_.y;
    data << z_;

    data << minimized_;
    data << muted_;
    data << enabled_;
    data << active_;
    data << flipped_;

    data << logger_level_;

    data << thread_id_;
    data << thread_name_;

    data << r_;
    data << g_;
    data << b_;

    data << dictionary;

    data << exec_mode_;
    data << exec_type_;

    YAML::Node yaml;
    parameter_state->writeYaml(yaml);
    data << yaml;
}

void NodeState::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    data >> max_frequency_;

    data >> label_;
    data >> pos_.x >> pos_.y;
    data >> z_;

    data >> minimized_;
    data >> muted_;
    data >> enabled_;
    data >> active_;
    data >> flipped_;

    data >> logger_level_;

    data >> thread_id_;
    data >> thread_name_;

    data >> r_;
    data >> g_;
    data >> b_;

    data >> dictionary;

    data >> exec_mode_;
    data >> exec_type_;

    YAML::Node yaml;
    data >> yaml;

    if (yaml.IsDefined()) {
        parameter_state->readYaml(yaml);
    }
}

bool NodeState::hasDictionaryEntry(const std::string& key) const
{
    return dictionary.find(key) != dictionary.end();
}

void NodeState::deleteDictionaryEntry(const std::string& key)
{
    dictionary.erase(key);
}

template <typename T>
T NodeState::getDictionaryEntry(const std::string& key) const
{
    return std::any_cast<T>(dictionary.at(key));
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
}  // namespace csapex

/// YAML
namespace YAML
{
Node convert<csapex::NodeState>::encode(const csapex::NodeState& rhs)
{
    Node n;
    rhs.writeYaml(n);
    return n;
}

bool convert<csapex::NodeState>::decode(const Node& node, csapex::NodeState& rhs)
{
    rhs.readYaml(node);
    return true;
}
}  // namespace YAML
