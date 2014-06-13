/// HEADER
#include <utils_param/range_parameter.h>

using namespace param;

RangeParameter::RangeParameter()
    : Parameter("noname", ParameterDescription())
{
}

RangeParameter::RangeParameter(const std::string &name, const ParameterDescription& description)
    : Parameter(name, description)
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


void RangeParameter::doSetValueFrom(const Parameter &other)
{
    const RangeParameter* range = dynamic_cast<const RangeParameter*>(&other);
    if(range) {
        bool change = false;
        if(value_.type() == typeid(int)) {
            change = boost::any_cast<int>(value_) != boost::any_cast<int>(range->value_);
        } else if(value_.type() == typeid(double)) {
            change = boost::any_cast<double>(value_) != boost::any_cast<double>(range->value_);
        }
        if(change) {
            value_ = range->value_;
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}


void RangeParameter::doClone(const Parameter &other)
{
    const RangeParameter* range = dynamic_cast<const RangeParameter*>(&other);
    if(range) {
        value_ = range->value_;
        min_ = range->min_;
        max_ = range->max_;
        def_ = range->def_;
        step_ = range->step_;
    } else {
        throw std::runtime_error("bad clone, invalid types");
    }
}

void RangeParameter::doWrite(YAML::Emitter& e) const
{
    e << YAML::Key << "type" << YAML::Value << "range";

    if(value_.type() == typeid(int)) {
        e << YAML::Key << "int" << YAML::Value << boost::any_cast<int> (value_);

    } else if(value_.type() == typeid(double)) {
        e << YAML::Key << "double" << YAML::Value << boost::any_cast<double> (value_);

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

void RangeParameter::doRead(const YAML::Node& n)
{
    if(!exists(n, "name")) {
        return;
    }

    n["name"] >> name_;

    if(exists(n, "int")) {
        value_ = __read<int>(n["int"]);

    } else if(exists(n, "double")) {
        value_ = __read<double>(n["double"]);
    }
}
