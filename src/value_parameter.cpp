/// HEADER
#include <utils_param/value_parameter.h>

using namespace param;

ValueParameter::ValueParameter()
    : Parameter("noname", ParameterDescription())
{
}

ValueParameter::ValueParameter(const std::string &name, const ParameterDescription &description)
    : Parameter(name, description)
{
}

ValueParameter::~ValueParameter()
{

}


const std::type_info& ValueParameter::type() const
{
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
    }

    return std::string("[value: ") + v.str()  + "]";
}

boost::any ValueParameter::get_unsafe() const
{
    return value_;
}


void ValueParameter::set_unsafe(const boost::any &v)
{
    value_ = v;
}


void ValueParameter::doSetValueFrom(const Parameter &other)
{
    const ValueParameter* value = dynamic_cast<const ValueParameter*>(&other);
    if(value) {
        bool change = false;
        if(value_.type() == typeid(int)) {
            change = boost::any_cast<int>(value_) != boost::any_cast<int>(value->value_);
        } else if(value_.type() == typeid(double)) {
            change = boost::any_cast<double>(value_) != boost::any_cast<double>(value->value_);
        } else if(value_.type() == typeid(bool)) {
            change = boost::any_cast<bool>(value_) != boost::any_cast<bool>(value->value_);
        } else if(value_.type() == typeid(std::string)) {
            change = boost::any_cast<std::string>(value_) != boost::any_cast<std::string>(value->value_);
        }
        if(change) {
            value_ = value->value_;
            triggerChange();
        }
    } else {
        try {
            value_ = access_unsafe(other);
            triggerChange();

        } catch(const std::exception& e) {
            throw std::runtime_error("bad setFrom, invalid types");
        }
    }
}

void ValueParameter::doClone(const Parameter &other)
{
    const ValueParameter* value = dynamic_cast<const ValueParameter*>(&other);
    if(value) {
        value_ = value->value_;
        def_ = value->def_;
    } else {
        throw std::runtime_error("bad clone, invalid types");
    }
}

void ValueParameter::doWrite(YAML::Emitter& e) const
{
    e << YAML::Key << "type" << YAML::Value << "value";

    if(value_.type() == typeid(int)) {
        e << YAML::Key << "int" << YAML::Value << boost::any_cast<int> (value_);

    } else if(value_.type() == typeid(double)) {
        e << YAML::Key << "double" << YAML::Value << boost::any_cast<double> (value_);

    } else if(value_.type() == typeid(bool)) {
        e << YAML::Key << "bool" << YAML::Value << boost::any_cast<bool> (value_);

    } else if(value_.type() == typeid(std::string)) {
        e << YAML::Key << "string" << YAML::Value << boost::any_cast<std::string> (value_);
    }
}

namespace {
template <typename T>
T __read(const YAML::Node& n) {
    T v;
    n >> v;
    return v;
}
}

void ValueParameter::doRead(const YAML::Node& n)
{
    if(!exists(n, "name")) {
        return;
    }

    n["name"] >> name_;

    if(exists(n, "int")) {
        value_ = __read<int>(n["int"]);

    } else if(exists(n, "double")) {
        value_ = __read<double>(n["double"]);

    } else if(exists(n, "bool")) {
        value_ = __read<bool>(n["bool"]);

    } else if(exists(n, "string")) {
        value_ = __read<std::string>(n["string"]);
    }
}
