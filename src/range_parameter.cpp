/// HEADER
#include <utils_param/range_parameter.h>

using namespace param;

RangeParameter::RangeParameter()
    : Parameter("noname")
{
}

RangeParameter::RangeParameter(const RangeParameter& rhs)
    : Parameter(rhs),
      value_(rhs.value_),
      min_(rhs.min_),
      max_(rhs.max_),
      def_(rhs.def_),
      step_(rhs.step_)
{
}

RangeParameter& RangeParameter::operator =(const RangeParameter& rhs)
{
    assert(name_ == rhs.name());
    value_ = rhs.value_;
    return *this;
}

RangeParameter::RangeParameter(const std::string &name)
    : Parameter(name)
{
}

RangeParameter::~RangeParameter()
{

}


const std::type_info& RangeParameter::type() const
{
    return value_.type();
}

std::string RangeParameter::toStringImpl() const
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

    return std::string("[ranged: ") + v.str()  + "]";
}

boost::any RangeParameter::get_unsafe() const
{
    return value_;
}


void RangeParameter::set_unsafe(const boost::any &v)
{
    value_ = v;
}


void RangeParameter::setFrom(const Parameter &other)
{
    const RangeParameter* range = dynamic_cast<const RangeParameter*>(&other);
    if(range) {
        value_ = range->value_;
        (*parameter_changed)(this);
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void RangeParameter::write(YAML::Emitter& e) const
{
    e << YAML::BeginMap;
    e << YAML::Key << "name" << YAML::Value << name();
    e << YAML::Key << "type" << YAML::Value << "range";

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

void RangeParameter::read(const YAML::Node& n)
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
