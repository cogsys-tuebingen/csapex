/// HEADER
#include <csapex/param/range_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

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
    Lock l = lock();
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


bool RangeParameter::set_unsafe(const boost::any &v)
{
    bool change = true;
    if(!value_.empty()) {
        if(v.type() == typeid(int)) {
            change = boost::any_cast<int>(value_) != boost::any_cast<int>(v);
        } else if(v.type() == typeid(double)) {
            change = boost::any_cast<double>(value_) != boost::any_cast<double>(v);
        }
    }
    if(change) {
        value_ = v;
        return true;
    }

    return false;
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
        def_min_ = range->def_min_;
        def_max_ = range->def_max_;
        def_value_ = range->def_value_;
        step_ = range->step_;
    } else {
        throw std::runtime_error("bad clone, invalid types");
    }
}

void RangeParameter::doSerialize(YAML::Node& n) const
{
    if(value_.type() == typeid(int)) {
        n["int"] = boost::any_cast<int> (value_);
        n["min"] = boost::any_cast<int> (min_);
        n["max"] = boost::any_cast<int> (max_);
        n["step"] = boost::any_cast<int> (step_);

    } else if(value_.type() == typeid(double)) {
        n["double"] = boost::any_cast<double> (value_);
        n["min"] = boost::any_cast<double> (min_);
        n["max"] = boost::any_cast<double> (max_);
        n["step"] = boost::any_cast<double> (step_);
    }
}

void RangeParameter::doDeserialize(const YAML::Node& n)
{
    if(!n["name"].IsDefined()) {
        return;
    }

    name_ = n["name"].as<std::string>();

    if(n["int"].IsDefined()) {
        value_ = n["int"].as<int>();
        if(n["min"].IsDefined())
            min_ = n["min"].as<int>();
        if(n["max"].IsDefined())
            max_ = n["max"].as<int>();
        if(n["step"].IsDefined())
            step_ = n["step"].as<int>();

    } else if(n["double"].IsDefined()) {
        value_ = n["double"].as<double>();
        if(n["min"].IsDefined())
            min_ = n["min"].as<double>();
        if(n["max"].IsDefined())
            max_ = n["max"].as<double>();
        if(n["step"].IsDefined())
            step_ = n["step"].as<double>();
    }

    if(def_min_.empty())
        def_min_ = min_;

    if(def_max_.empty())
        def_max_ = max_;

    if(def_value_.empty())
        def_value_ = value_;
}
