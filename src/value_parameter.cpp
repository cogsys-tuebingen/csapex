/// HEADER
#include <utils_param/value_parameter.h>

using namespace param;

ValueParameter::ValueParameter()
    : Parameter("noname")
{
}

ValueParameter::ValueParameter(const std::string &name)
    : Parameter(name)
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


void ValueParameter::doSetFrom(const Parameter &other)
{
    const ValueParameter* range = dynamic_cast<const ValueParameter*>(&other);
    if(range) {
        value_ = range->value_;
        triggerChange();
    } else {
        try {
            value_ = access_unsafe(other);
            triggerChange();

        } catch(const std::exception& e) {
            throw std::runtime_error("bad setFrom, invalid types");
        }
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
