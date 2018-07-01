/// HEADER
#include <csapex/param/angle_parameter.h>

/// PROJECT
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/param/value_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <boost/any.hpp>

CSAPEX_REGISTER_PARAMETER_SERIALIZER(AngleParameter)

using namespace csapex;
using namespace param;

AngleParameter::AngleParameter() : ParameterImplementation("angle", ParameterDescription()), angle_(0.0), min_(-M_PI), max_(M_PI)
{
}

AngleParameter::AngleParameter(const std::string& name, const ParameterDescription& description, double angle, double min, double max)
  : ParameterImplementation(name, description), angle_(angle), min_(min), max_(max)
{
    set(angle_);
}

const std::type_info& AngleParameter::type() const
{
    Lock l = lock();
    return typeid(double);
}

std::string AngleParameter::toStringImpl() const
{
    return std::string("[angle: ") + std::to_string(angle_) + "rad, " + std::to_string(angle_ / M_PI * 180.0) + "°]";
}

void AngleParameter::get_unsafe(boost::any& out) const
{
    out = angle_;
}

bool AngleParameter::set_unsafe(const boost::any& v)
{
    double a = boost::any_cast<double>(v);
    if (a < min_ || a >= max_) {
        throw std::out_of_range(std::string("angle ") + std::to_string(a) + " is not in the valid interval [" + std::to_string(min_) + ", " + std::to_string(max_) + "]");
    }

    if (a != angle_) {
        angle_ = a;
        return true;
    }

    return false;
}

double AngleParameter::min() const
{
    return min_;
}
double AngleParameter::max() const
{
    return max_;
}

void AngleParameter::cloneDataFrom(const Clonable& other)
{
    if (const AngleParameter* angle = dynamic_cast<const AngleParameter*>(&other)) {
        if (angle_ != angle->angle_) {
            *this = *angle;
            triggerChange();
        }
    } else if (const ValueParameter* value = dynamic_cast<const ValueParameter*>(&other)) {
        if (angle_ != value->as<double>()) {
            angle_ = value->as<double>();
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void AngleParameter::doSerialize(YAML::Node& n) const
{
    n["value"] = angle_;
    n["min"] = min_;
    n["max"] = max_;
}

void AngleParameter::doDeserialize(const YAML::Node& n)
{
    if (!n["value"].IsDefined()) {
        return;
    }

    double a = n["value"].as<double>();
    set(a);
    // min_ = n["min"].as<double>();
    // max_ = n["max"].as<double>();
}

void AngleParameter::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Parameter::serialize(data, version);

    data << angle_;
    data << min_;
    data << max_;
}

void AngleParameter::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Parameter::deserialize(data, version);

    data >> angle_;
    data >> min_;
    data >> max_;
}
