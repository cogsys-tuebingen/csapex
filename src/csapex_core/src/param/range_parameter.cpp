/// HEADER
#include <csapex/param/range_parameter.h>

/// PROJECT
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <iostream>

CSAPEX_REGISTER_PARAMETER_SERIALIZER(RangeParameter)

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
    bool value_change = false;
    if (value_.type() == typeid(int)) {
        value_change = boost::any_cast<int>(value_) != boost::any_cast<int>(range.value_);
        step_ = range::limitStep(boost::any_cast<int>(range.min_), boost::any_cast<int>(range.max_), boost::any_cast<int>(range.step_));
    } else if (value_.type() == typeid(double)) {
        value_change = boost::any_cast<double>(value_) != boost::any_cast<double>(range.value_);
        step_ = range::limitStep(boost::any_cast<double>(range.min_), boost::any_cast<double>(range.max_), boost::any_cast<double>(range.step_));
    }
    value_ = range.value_;
    min_ = range.min_;
    max_ = range.max_;
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
        v << boost::any_cast<int>(value_);

    } else if (value_.type() == typeid(double)) {
        v << boost::any_cast<double>(value_);
    }

    return std::string("[ranged: ") + v.str() + "]";
}

void RangeParameter::get_unsafe(boost::any& out) const
{
    out = value_;
}

bool RangeParameter::set_unsafe(const boost::any& v)
{
    bool change = true;
    if (!value_.empty()) {
        if (v.type() == typeid(int)) {
            change = boost::any_cast<int>(value_) != boost::any_cast<int>(v);
        } else if (v.type() == typeid(double)) {
            change = boost::any_cast<double>(value_) != boost::any_cast<double>(v);
        }
    }
    if (change) {
        value_ = v;
        return true;
    }

    return false;
}

void RangeParameter::cloneDataFrom(const Clonable& other)
{
    if (const RangeParameter* range = dynamic_cast<const RangeParameter*>(&other)) {
        *this = *range;
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void RangeParameter::doSerialize(YAML::Node& n) const
{
    if (value_.type() == typeid(int)) {
        n["int"] = boost::any_cast<int>(value_);
        n["min"] = boost::any_cast<int>(min_);
        n["max"] = boost::any_cast<int>(max_);
        n["step"] = boost::any_cast<int>(step_);

    } else if (value_.type() == typeid(double)) {
        n["double"] = boost::any_cast<double>(value_);
        n["min"] = boost::any_cast<double>(min_);
        n["max"] = boost::any_cast<double>(max_);
        n["step"] = boost::any_cast<double>(step_);
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
            step_ = range::limitStep(boost::any_cast<int>(min_), boost::any_cast<int>(max_), n["step"].as<int>());

    } else if (n["double"].IsDefined()) {
        value_ = n["double"].as<double>();
        if (n["min"].IsDefined())
            min_ = n["min"].as<double>();
        if (n["max"].IsDefined())
            max_ = n["max"].as<double>();
        if (n["step"].IsDefined())
            step_ = range::limitStep(boost::any_cast<double>(min_), boost::any_cast<double>(max_), n["step"].as<double>());
    }

    if (def_min_.empty())
        def_min_ = min_;

    if (def_max_.empty())
        def_max_ = max_;

    if (def_value_.empty())
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
