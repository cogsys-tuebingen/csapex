/// HEADER
#include <csapex/param/bitset_parameter.h>

/// PROJECT
#include <csapex/param/register_parameter.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <any>

CSAPEX_REGISTER_PARAM(BitSetParameter)

using namespace csapex;
using namespace param;

BitSetParameter::BitSetParameter() : ParameterImplementation("noname", ParameterDescription())
{
}

BitSetParameter::BitSetParameter(const std::string& name, const ParameterDescription& description, int def) : ParameterImplementation(name, description), value_(def)
{
}

BitSetParameter::~BitSetParameter()
{
}

bool BitSetParameter::accepts(const std::type_info& type) const
{
    return type == typeid(int) || type == typeid(std::pair<std::string, bool>);
}

void BitSetParameter::setByName(const std::string& name)
{
    for (std::map<std::string, int>::iterator it = set_.begin(); it != set_.end(); ++it) {
        if (it->first == name) {
            value_ = it->second;
            triggerChange();
            return;
        }
    }

    throw std::runtime_error(std::string("no such parameter: ") + name);
}

void BitSetParameter::setBitSet(const std::map<std::string, int>& set)
{
    set_ = set;
    scope_changed(this);
}

std::map<std::string, int> BitSetParameter::getBitSet() const
{
    return set_;
}

void BitSetParameter::clear()
{
    value_ = 0;
    triggerChange();
}

void BitSetParameter::setBits(const std::vector<std::string>& elements, bool silent)
{
    bool change = false;

    for (std::map<std::string, int>::iterator set_it = set_.begin(); set_it != set_.end(); ++set_it) {
        bool found = false;
        const std::string& e = set_it->first;
        for (std::vector<std::string>::const_iterator e_it = elements.begin(); e_it != elements.end(); ++e_it) {
            if (e == *e_it) {
                found = true;
                break;
            }
        }
        if (found) {
            if (!isSet(e)) {
                setBit(e, true);
                change = true;
            }
        } else {
            if (isSet(e)) {
                clearBit(e, true);
                change = true;
            }
        }
    }

    if (change && !silent) {
        triggerChange();
    }
}

void BitSetParameter::setBitTo(const std::string& element, bool set, bool silent)
{
    for (std::map<std::string, int>::iterator it = set_.begin(); it != set_.end(); ++it) {
        if (it->first == element) {
            if (set) {
                value_ |= it->second;
            } else {
                value_ &= ~(it->second);
            }

            if (!silent) {
                triggerChange();
            }
            return;
        }
    }
}

void BitSetParameter::setBit(const std::string& element, bool silent)
{
    setBitTo(element, true, silent);
}

void BitSetParameter::clearBit(const std::string& element, bool silent)
{
    setBitTo(element, false, silent);
}

bool BitSetParameter::isSet(const std::string& element) const
{
    for (std::map<std::string, int>::const_iterator it = set_.begin(); it != set_.end(); ++it) {
        if (it->first == element) {
            int target = it->second;

            return (value_ & target) == target;
        }
    }
    return false;
}

int BitSetParameter::noParameters() const
{
    return set_.size();
}

std::string BitSetParameter::getName(int idx) const
{
    std::map<std::string, int>::const_iterator i = set_.begin();
    std::advance(i, idx);
    return i->first;
}

std::string BitSetParameter::getName() const
{
    throw std::runtime_error("cannot get the name for parameter '" + name() + "'");
}

const std::type_info& BitSetParameter::type() const
{
    return typeid(int);
}

std::string BitSetParameter::toStringImpl() const
{
    return std::string("[bitset: ") + "]";
}

void BitSetParameter::get_unsafe(std::any& out) const
{
    out = value_;
}

bool BitSetParameter::set_unsafe(const std::any& v)
{
    if (v.type() == typeid(int)) {
        int val = std::any_cast<int>(v);
        if (val != value_) {
            value_ = val;
            return true;
        }
    } else if (v.type() == typeid(std::pair<std::string, bool>)) {
        auto pair = std::any_cast<std::pair<std::string, bool>>(v);
        setBitTo(pair.first, pair.second);
        return true;
    }

    return false;
}

bool BitSetParameter::cloneDataFrom(const Clonable& other)
{
    if (const BitSetParameter* range = dynamic_cast<const BitSetParameter*>(&other)) {
        bool value_changed = value_ != range->value_;
        *this = *range;
        if (value_changed) {
            triggerChange();
        }
        return true;

    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }

    return false;
}

void BitSetParameter::doSerialize(YAML::Node& n) const
{
    n["int"] = std::any_cast<int>(value_);
    n["values"] = set_;
}

void BitSetParameter::doDeserialize(const YAML::Node& n)
{
    if (n["int"].IsDefined()) {
        value_ = n["int"].as<int>();
    }

    if (n["values"].IsDefined()) {
        set_ = n["values"].as<std::map<std::string, int>>();
        int full_mask = 0;
        for (auto& entry : set_)
            full_mask |= entry.second;

        value_ &= full_mask;
    }
}

void BitSetParameter::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Parameter::serialize(data, version);

    data << value_;
    data << set_;
    data << def_;
}

void BitSetParameter::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Parameter::deserialize(data, version);

    data >> value_;
    data >> set_;
    data >> def_;
}
