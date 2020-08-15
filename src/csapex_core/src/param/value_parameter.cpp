/// HEADER
#include <csapex/param/value_parameter.h>

/// PROJECT
#include <csapex/param/register_parameter.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <iostream>
#include <iomanip>

CSAPEX_REGISTER_PARAM(ValueParameter)

using namespace csapex;
using namespace param;

ValueParameter::ValueParameter() : ParameterImplementation("noname", ParameterDescription())
{
}

ValueParameter::ValueParameter(const std::string& name, const ParameterDescription& description) : ParameterImplementation(name, description)
{
}

ValueParameter::~ValueParameter()
{
}

const std::type_info& ValueParameter::type() const
{
    Lock l = lock();
    return value_.type();
}

std::string ValueParameter::toStringImpl() const
{
    std::stringstream v;

    if (value_.type() == typeid(int)) {
        v << std::any_cast<int>(value_);

    } else if (value_.type() == typeid(double)) {
        v << std::showpoint << std::setprecision(5) << std::any_cast<double>(value_);

    } else if (value_.type() == typeid(bool)) {
        v << (std::any_cast<bool>(value_) ? "true" : "false");

    } else if (value_.type() == typeid(std::string)) {
        v << std::any_cast<std::string>(value_);

    } else if (value_.type() == typeid(long)) {
        v << std::any_cast<long>(value_);
    }

    return std::string("[value: ") + v.str() + "]";
}

void ValueParameter::get_unsafe(std::any& out) const
{
    out = value_;
}

bool ValueParameter::set_unsafe(const std::any& v)
{
    bool change = true;
    if (any_has_value(value_)) {
        if (v.type() == typeid(int)) {
            change = std::any_cast<int>(value_) != std::any_cast<int>(v);
        } else if (v.type() == typeid(double)) {
            change = std::any_cast<double>(value_) != std::any_cast<double>(v);
        } else if (v.type() == typeid(bool)) {
            change = std::any_cast<bool>(value_) != std::any_cast<bool>(v);
        } else if (v.type() == typeid(std::string)) {
            change = std::any_cast<std::string>(value_) != std::any_cast<std::string>(v);
        } else if (v.type() == typeid(long)) {
            change = std::any_cast<long>(value_) != std::any_cast<long>(v);
        }
    }
    if (change) {
        value_ = v;
        return true;
    }

    return false;
}

bool ValueParameter::cloneDataFrom(const Clonable& other)
{
    if (const ValueParameter* value = dynamic_cast<const ValueParameter*>(&other)) {
        bool value_change = true;
        if (value_.type() == value->value_.type()) {
            if (value_.type() == typeid(int)) {
                value_change = std::any_cast<int>(value_) != std::any_cast<int>(value->value_);
            } else if (value_.type() == typeid(double)) {
                value_change = std::any_cast<double>(value_) != std::any_cast<double>(value->value_);
            } else if (value_.type() == typeid(bool)) {
                value_change = std::any_cast<bool>(value_) != std::any_cast<bool>(value->value_);
            } else if (value_.type() == typeid(std::string)) {
                value_change = std::any_cast<std::string>(value_) != std::any_cast<std::string>(value->value_);
            } else if (value_.type() == typeid(long)) {
                value_change = std::any_cast<long>(value_) != std::any_cast<long>(value->value_);
            }
        }
        *this = *value;
        if (value_change) {
            triggerChange();
        }
        return true;

    } else if (const Parameter* param = dynamic_cast<const Parameter*>(&other)) {
        try {
            access_unsafe(*param, value_);
            triggerChange();
            return true;

        } catch (const std::exception& e) {
            throw std::runtime_error("bad setFrom, invalid types");
        }
    }

    return false;
}

namespace
{
template <typename T>
T __read(const YAML::Node& n)
{
    return n.as<T>();
}
}  // namespace

void ValueParameter::doDeserialize(const YAML::Node& n)
{
    if (n["int"].IsDefined()) {
        value_ = __read<int>(n["int"]);

    } else if (n["double"].IsDefined()) {
        value_ = __read<double>(n["double"]);

    } else if (n["bool"].IsDefined()) {
        value_ = __read<bool>(n["bool"]);

    } else if (n["string"].IsDefined()) {
        value_ = __read<std::string>(n["string"]);

    } else if (n["long"].IsDefined()) {
        value_ = __read<long>(n["long"]);
    }
}

void ValueParameter::doSerialize(YAML::Node& n) const
{
    if (value_.type() == typeid(int)) {
        n["int"] = std::any_cast<int>(value_);

    } else if (value_.type() == typeid(double)) {
        n["double"] = std::any_cast<double>(value_);

    } else if (value_.type() == typeid(bool)) {
        n["bool"] = std::any_cast<bool>(value_);

    } else if (value_.type() == typeid(std::string)) {
        n["string"] = std::any_cast<std::string>(value_);

    } else if (value_.type() == typeid(long)) {
        n["long"] = std::any_cast<long>(value_);
    }
}

void ValueParameter::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Parameter::serialize(data, version);

    data << value_;
    data << def_;
}

void ValueParameter::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Parameter::deserialize(data, version);

    data >> value_;
    data >> def_;
}
