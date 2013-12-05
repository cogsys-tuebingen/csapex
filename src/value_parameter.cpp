/// HEADER
#include <utils_param/value_parameter.h>

using namespace param;

ValueParameter::ValueParameter()
    : Parameter("noname")
{
}

ValueParameter::ValueParameter(const ValueParameter& rhs)
    : Parameter(rhs),
      value_(rhs.value_),
      def_(rhs.def_)
{
}

ValueParameter& ValueParameter::operator =(const ValueParameter& rhs)
{
    assert(name_ == rhs.name());
    value_ = rhs.value_;
    return *this;
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


void ValueParameter::setFrom(const Parameter &other)
{
    const ValueParameter* range = dynamic_cast<const ValueParameter*>(&other);
    if(range) {
        value_ = range->value_;
        (*parameter_changed)(this);
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void ValueParameter::write(YAML::Emitter& e) const
{
    e << YAML::BeginMap;
    e << YAML::Key << "name" << YAML::Value << name();
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

    e << YAML::EndMap;
}

namespace {
template <typename T>
T __read(const YAML::Node& n) {
    T v;
    n >> v;
    return v;
}
}

void ValueParameter::read(const YAML::Node& n)
{
    if(!n.FindValue("name")) {
        return;
    }

    n["name"] >> name_;

    if(n.FindValue("int")) {
        value_ = __read<int>(n["int"]);

    } else if(n.FindValue("double")) {
        value_ = __read<double>(n["double"]);

    } else if(n.FindValue("bool")) {
        value_ = __read<bool>(n["bool"]);

    } else if(n.FindValue("string")) {
        value_ = __read<std::string>(n["string"]);
    }
}
