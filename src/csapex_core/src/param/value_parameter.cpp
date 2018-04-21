/// HEADER
#include <csapex/param/value_parameter.h>

/// PROJECT
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <iostream>

CSAPEX_REGISTER_PARAMETER_SERIALIZER(ValueParameter)

using namespace csapex;
using namespace param;

ValueParameter::ValueParameter()
    : ParameterImplementation("noname", ParameterDescription())
{
}

ValueParameter::ValueParameter(const std::string &name, const ParameterDescription &description)
    : ParameterImplementation(name, description)
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

    if(value_.type() == typeid(int)) {
        v << boost::any_cast<int> (value_);

    } else if(value_.type() == typeid(double)) {
        v << boost::any_cast<double> (value_);

    } else if(value_.type() == typeid(bool)) {
        v << boost::any_cast<bool> (value_);

    } else if(value_.type() == typeid(std::string)) {
        v << boost::any_cast<std::string> (value_);

    } else if(value_.type() == typeid(long)) {
        v << boost::any_cast<long> (value_);
    }

    return std::string("[value: ") + v.str()  + "]";
}

void ValueParameter::get_unsafe(boost::any& out) const
{
    out = value_;
}


bool ValueParameter::set_unsafe(const boost::any &v)
{
    bool change = true;
    if(!value_.empty()) {
        if(v.type() == typeid(int)) {
            change = boost::any_cast<int>(value_) != boost::any_cast<int>(v);
        } else if(v.type() == typeid(double)) {
            change = boost::any_cast<double>(value_) != boost::any_cast<double>(v);
        } else if(v.type() == typeid(bool)) {
            change = boost::any_cast<bool>(value_) != boost::any_cast<bool>(v);
        } else if(v.type() == typeid(std::string)) {
            change = boost::any_cast<std::string>(value_) != boost::any_cast<std::string>(v);
        } else if(v.type() == typeid(long)) {
            change = boost::any_cast<long>(value_) != boost::any_cast<long>(v);
        }
    }
    if(change) {
        value_ = v;
        return true;
    }

    return false;
}


void ValueParameter::cloneDataFrom(const Clonable &other)
{
    if(const ValueParameter* value = dynamic_cast<const ValueParameter*>(&other)) {
        bool change = false;
        if(value_.type() == typeid(int)) {
            change = boost::any_cast<int>(value_) != boost::any_cast<int>(value->value_);
        } else if(value_.type() == typeid(double)) {
            change = boost::any_cast<double>(value_) != boost::any_cast<double>(value->value_);
        } else if(value_.type() == typeid(bool)) {
            change = boost::any_cast<bool>(value_) != boost::any_cast<bool>(value->value_);
        } else if(value_.type() == typeid(std::string)) {
            change = boost::any_cast<std::string>(value_) != boost::any_cast<std::string>(value->value_);
        } else if(value_.type() == typeid(long)) {
            change = boost::any_cast<long>(value_) != boost::any_cast<long>(value->value_);
        } else {
            change = true;
        }
        if(change) {
            *this = *value;
            triggerChange();
        }
    } else if(const Parameter* param = dynamic_cast<const Parameter*>(&other)) {
        try {
            access_unsafe(*param, value_);
            triggerChange();

        } catch(const std::exception& e) {
            throw std::runtime_error("bad setFrom, invalid types");
        }
    }
}

namespace {
template <typename T>
T __read(const YAML::Node& n) {
    return n.as<T>();
}
}

void ValueParameter::doDeserialize(const YAML::Node& n)
{
    if(n["int"].IsDefined()) {
        value_ = __read<int>(n["int"]);

    } else if(n["double"].IsDefined()) {
        value_ = __read<double>(n["double"]);

    } else if(n["bool"].IsDefined()) {
        value_ = __read<bool>(n["bool"]);

    } else if(n["string"].IsDefined()) {
        value_ = __read<std::string>(n["string"]);

    } else if(n["long"].IsDefined()) {
        value_ = __read<long>(n["long"]);
    }
}


void ValueParameter::doSerialize(YAML::Node& n) const
{
    if(value_.type() == typeid(int)) {
        n["int"] = boost::any_cast<int> (value_);

    } else if(value_.type() == typeid(double)) {
        n["double"] = boost::any_cast<double> (value_);

    } else if(value_.type() == typeid(bool)) {
        n["bool"] = boost::any_cast<bool> (value_);

    } else if(value_.type() == typeid(std::string)) {
        n["string"] = boost::any_cast<std::string> (value_);

    } else if(value_.type() == typeid(long)) {
        n["long"] = boost::any_cast<long> (value_);
    }
}




void ValueParameter::serialize(SerializationBuffer &data) const
{
    Parameter::serialize(data);

    data << value_;
    data << def_;
}

void ValueParameter::deserialize(const SerializationBuffer& data)
{
    Parameter::deserialize(data);

    data >> value_;
    data >> def_;
}

