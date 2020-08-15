/// HEADER
#include <csapex/param/range_parameter.h>

/// PROJECT
#include <csapex/param/register_parameter.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <iostream>

CSAPEX_REGISTER_PARAM(RangeParameter)

using namespace csapex;
using namespace param;

RangeParameter::RangeParameter() : ParameterImplementation("noname", ParameterDescription())
{
}

RangeParameter::RangeParameter(const std::string& name, const ParameterDescription& description) : ParameterImplementation(name, description)
{
}

RangeParameter::~RangeParameter()
{
}

RangeParameter& RangeParameter::operator=(const RangeParameter& range)
{
    Parameter::operator=(static_cast<const Parameter&>(range));

    bool value_change = false;
    if (!any_has_value(value_) || value_.type() != range.value_.type()) {
        value_change = true;
        step_ = range.step_;
    } else {
        if (value_.type() == typeid(int)) {
            value_change = std::any_cast<int>(value_) != std::any_cast<int>(range.value_);
            step_ = range::limitStep(std::any_cast<int>(range.min_), std::any_cast<int>(range.max_), std::any_cast<int>(range.step_));
        } else if (value_.type() == typeid(double)) {
            value_change = std::any_cast<double>(value_) != std::any_cast<double>(range.value_);
            step_ = range::limitStep(std::any_cast<double>(range.min_), std::any_cast<double>(range.max_), std::any_cast<double>(range.step_));
        }
    }
    value_ = range.value_;
    min_ = range.min_;
    max_ = range.max_;
    def_value_ = range.def_value_;
    def_min_ = range.def_min_;
    def_max_ = range.def_max_;
    if (value_change) {
        triggerChange();
    }

    return *this;
}

const std::type_info& RangeParameter::type() const
{
    Lock l = lock();
    return value_.type();
}

std::string RangeParameter::toStringImpl() const
{
    std::stringstream v;

    if (value_.type() == typeid(int)) {
        v << std::any_cast<int>(value_);

    } else if (value_.type() == typeid(double)) {
        v << std::any_cast<double>(value_);
    }

    return std::string("[ranged: ") + v.str() + "]";
}

void RangeParameter::get_unsafe(std::any& out) const
{
    out = value_;
}

bool RangeParameter::set_unsafe(const std::any& v)
{
    bool change = true;
    if (any_has_value(value_)) {
        if (v.type() == typeid(int)) {
            change = std::any_cast<int>(value_) != std::any_cast<int>(v);
        } else if (v.type() == typeid(double)) {
            change = std::any_cast<double>(value_) != std::any_cast<double>(v);
        }
    }
    if (change) {
        value_ = v;
        return true;
    }

    return false;
}

bool RangeParameter::cloneDataFrom(const Clonable& other)
{
    if (const RangeParameter* range = dynamic_cast<const RangeParameter*>(&other)) {
        *this = *range;
        return true;

    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }

    return false;
}

void RangeParameter::doSerialize(YAML::Node& n) const
{
    if (value_.type() == typeid(int)) {
        n["int"] = std::any_cast<int>(value_);
        n["min"] = std::any_cast<int>(min_);
        n["max"] = std::any_cast<int>(max_);
        n["step"] = std::any_cast<int>(step_);

    } else if (value_.type() == typeid(double)) {
        n["double"] = std::any_cast<double>(value_);
        n["min"] = std::any_cast<double>(min_);
        n["max"] = std::any_cast<double>(max_);
        n["step"] = std::any_cast<double>(step_);
    }
}

void RangeParameter::doDeserialize(const YAML::Node& n)
{
    if (n["int"].IsDefined()) {
        value_ = n["int"].as<int>();
        if (n["min"].IsDefined())
            min_ = n["min"].as<int>();
        if (n["max"].IsDefined())
            max_ = n["max"].as<int>();
        if (n["step"].IsDefined())
            step_ = range::limitStep(std::any_cast<int>(min_), std::any_cast<int>(max_), n["step"].as<int>());

    } else if (n["double"].IsDefined()) {
        value_ = n["double"].as<double>();
        if (n["min"].IsDefined())
            min_ = n["min"].as<double>();
        if (n["max"].IsDefined())
            max_ = n["max"].as<double>();
        if (n["step"].IsDefined())
            step_ = range::limitStep(std::any_cast<double>(min_), std::any_cast<double>(max_), n["step"].as<double>());
    }

    if (!any_has_value(def_min_))
        def_min_ = min_;

    if (!any_has_value(def_max_))
        def_max_ = max_;

    if (!any_has_value(def_value_))
        def_value_ = value_;
}

void RangeParameter::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Parameter::serialize(data, version);

    data << value_;
    data << min_;
    data << max_;
    data << def_value_;
    data << def_min_;
    data << def_max_;
    data << step_;
}

void RangeParameter::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Parameter::deserialize(data, version);

    data >> value_;
    data >> min_;
    data >> max_;
    data >> def_value_;
    data >> def_min_;
    data >> def_max_;
    data >> step_;
}

namespace csapex
{
namespace param
{
namespace range
{
template <>
double limitStep<double>(const double min, const double max, const double step)
{
    double range = max - min;
    long long necessary_intervals = (range) / step + 1;

    long max_intervals = std::numeric_limits<int>::max();

    if (necessary_intervals >= max_intervals) {
        std::cerr << "step size " << step << " is to small with range [" << min << ", " << max << "]"
                  << ", would take " << necessary_intervals << " intervals, which is larger than std::numeric_limits<int>::max() = " << max_intervals << "!" << std::endl;
        double min_step_size = (range / max_intervals);
        double res = step;
        while (res < min_step_size) {
            res *= 10.0;
        }
        std::cerr << "increasing step size to " << res << std::endl;
        return limitStep(min, max, res);
    }
    return step;
}

template <>
int limitStep(const int min, const int max, const int step)
{
    if (step == 0) {
        std::cerr << "step cannot be 0! setting to 1" << std::endl;
        return 1;
    }
    return step;
}

}  // namespace range
}  // namespace param
}  // namespace csapex
