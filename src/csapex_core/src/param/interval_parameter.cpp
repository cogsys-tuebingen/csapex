/// HEADER
#include <csapex/param/interval_parameter.h>

/// PROJECT
#include <csapex/param/register_parameter.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#undef NDEBUG
#include <assert.h>

CSAPEX_REGISTER_PARAM(IntervalParameter)

using namespace csapex;
using namespace param;

IntervalParameter::IntervalParameter() : ParameterImplementation("noname", ParameterDescription())
{
}

IntervalParameter::IntervalParameter(const std::string& name, const ParameterDescription& description) : ParameterImplementation(name, description)
{
}

IntervalParameter::~IntervalParameter()
{
}

IntervalParameter& IntervalParameter::operator=(const IntervalParameter& interval)
{
    Parameter::operator=(static_cast<const Parameter&>(interval));

    bool change = false;
    if (!any_has_value(values_.first) || !any_has_value(values_.second)) {
        change = true;

    } else {
        if (type() == typeid(std::pair<int, int>)) {
            change = std::any_cast<int>(values_.first) != std::any_cast<int>(interval.values_.first) || std::any_cast<int>(values_.second) != std::any_cast<int>(interval.values_.second);
        } else if (type() == typeid(std::pair<double, double>)) {
            change = std::any_cast<double>(values_.first) != std::any_cast<double>(interval.values_.first) ||
                     std::any_cast<double>(values_.second) != std::any_cast<double>(interval.values_.second);
        }
    }
    values_ = interval.values_;

    def_ = interval.def_;

    if (any_has_value(interval.max_)) {
        max_ = interval.max_;
    }
    if (any_has_value(interval.min_)) {
        min_ = interval.min_;
    }
    if (any_has_value(interval.step_)) {
        step_ = interval.step_;
    }

    if (change) {
        triggerChange();
    }

    return *this;
}

bool IntervalParameter::accepts(const std::type_info& type) const
{
    if (!any_has_value(values_.first)) {
        return type == typeid(std::pair<int, int>) || type == typeid(std::pair<double, double>);
    } else {
        return Parameter::accepts(type);
    }
}

const std::type_info& IntervalParameter::type() const
{
    Lock l = lock();
    if (values_.first.type() == typeid(int)) {
        return typeid(std::pair<int, int>);
    } else if (values_.first.type() == typeid(double)) {
        return typeid(std::pair<double, double>);
    } else {
        throw std::logic_error("unknown type");
    }
}

std::string IntervalParameter::toStringImpl() const
{
    Lock l = lock();
    std::stringstream v;
    v << "[interval: ";

    if (values_.first.type() == typeid(int)) {
        v << std::any_cast<int>(values_.first) << " : " << std::any_cast<int>(values_.second);

    } else if (values_.first.type() == typeid(double)) {
        v << std::any_cast<double>(values_.first) << " : " << std::any_cast<double>(values_.second);
    }

    v << "]";

    return v.str();
}

void IntervalParameter::get_unsafe(std::any& out) const
{
    Lock l = lock();
    if (is<std::pair<int, int> >()) {
        out = std::make_pair(std::any_cast<int>(values_.first), std::any_cast<int>(values_.second));
    } else {
        out = std::make_pair(std::any_cast<double>(values_.first), std::any_cast<double>(values_.second));
    }
}

bool IntervalParameter::set_unsafe(const std::any& v)
{
    Lock l = lock();
    if (v.type() == typeid(std::pair<int, int>)) {
        auto val = std::any_cast<std::pair<int, int> >(v);
        if (!any_has_value(values_.first)) {
            values_ = val;
            return true;
        }
        if (std::any_cast<int>(values_.first) != val.first || std::any_cast<int>(values_.second) != val.second) {
            values_ = val;
            return true;
        }
    } else {
        auto val = std::any_cast<std::pair<double, double> >(v);
        if (!any_has_value(values_.first)) {
            values_ = val;
            return true;
        }
        if (std::any_cast<double>(values_.first) != val.first || std::any_cast<double>(values_.second) != val.second) {
            values_ = val;
            return true;
        }
    }

    return false;
}

bool IntervalParameter::cloneDataFrom(const Clonable& other)
{
    Lock l = lock();
    if (const IntervalParameter* interval = dynamic_cast<const IntervalParameter*>(&other)) {
        *this = *interval;
        return true;

    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }

    return false;
}

void IntervalParameter::doSerialize(YAML::Node& n) const
{
    Lock l = lock();
    if (values_.first.type() == typeid(int)) {
        n["int"][0] = std::any_cast<int>(values_.first);
        n["int"][1] = std::any_cast<int>(values_.second);
        if (any_has_value(min_))
            n["min"] = std::any_cast<int>(min_);
        if (any_has_value(max_))
            n["max"] = std::any_cast<int>(max_);
        if (any_has_value(step_))
            n["step"] = std::any_cast<int>(step_);

    } else if (values_.first.type() == typeid(double)) {
        n["double"][0] = std::any_cast<double>(values_.first);
        n["double"][1] = std::any_cast<double>(values_.second);
        if (any_has_value(min_))
            n["min"] = std::any_cast<double>(min_);
        if (any_has_value(max_))
            n["max"] = std::any_cast<double>(max_);
        if (any_has_value(step_))
            n["step"] = std::any_cast<double>(step_);
    }
}

namespace
{
template <typename T>
std::pair<T, T> __read(const YAML::Node& n)
{
    std::pair<T, T> v;
    assert(n.Type() == YAML::NodeType::Sequence);
    v.first = n[0].as<T>();
    v.second = n[1].as<T>();
    return v;
}
}  // namespace

void IntervalParameter::doDeserialize(const YAML::Node& n)
{
    if (n["int"].IsDefined()) {
        values_ = __read<int>(n["int"]);
        if (n["min"].IsDefined())
            min_ = n["min"].as<int>();
        if (n["max"].IsDefined())
            max_ = n["max"].as<int>();
        if (n["step"].IsDefined())
            step_ = n["step"].as<int>();

    } else if (n["double"].IsDefined()) {
        values_ = __read<double>(n["double"]);
        if (n["min"].IsDefined())
            min_ = n["min"].as<double>();
        if (n["max"].IsDefined())
            max_ = n["max"].as<double>();
        if (n["step"].IsDefined())
            step_ = n["step"].as<double>();

    } else {
        throw std::runtime_error("cannot read interval");
    }
}

void IntervalParameter::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Parameter::serialize(data, version);

    data << values_;
    data << min_;
    data << max_;
    data << def_;
    data << step_;
}

void IntervalParameter::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Parameter::deserialize(data, version);

    data >> values_;
    data >> min_;
    data >> max_;
    data >> def_;
    data >> step_;
}
