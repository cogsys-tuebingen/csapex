/// HEADER
#include <utils_param/angle_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

using namespace param;

AngleParameter::AngleParameter()
    : Parameter("angle", ParameterDescription())
{
}

AngleParameter::AngleParameter(const std::string &name, const ParameterDescription& description, double angle, double min, double max)
    : Parameter(name, description), angle_(angle), min_(min), max_(max)
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
    return std::string("[angle: ") + std::to_string(angle_ ) + "rad, " + std::to_string(angle_ / M_PI * 180.0) +  "°]";
}

boost::any AngleParameter::get_unsafe() const
{
    return angle_;
}


bool AngleParameter::set_unsafe(const boost::any &v)
{
    double a = boost::any_cast<double> (v);
    if(a < min_ || a >= max_) {
        throw std::out_of_range("angle is not in the valid interval");
    }

    if(a != angle_) {
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

void AngleParameter::doSetValueFrom(const Parameter &other)
{
    const AngleParameter* angle = dynamic_cast<const AngleParameter*>(&other);
    if(angle) {
        if(angle_ != angle->angle_) {
            angle_ = angle->angle_;
            min_ = angle->min_;
            max_ = angle->max_;
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void AngleParameter::doClone(const Parameter &other)
{
    const AngleParameter* angle = dynamic_cast<const AngleParameter*>(&other);
    if(angle) {
        angle_ = angle->angle_;
        min_ = angle->min_;
        max_ = angle->max_;
    } else {
        throw std::runtime_error("bad clone, invalid types");
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
    if(!n["name"].IsDefined()) {
        return;
    }

    name_ = n["name"].as<std::string>();

    if(!n["value"].IsDefined()) {
        return;
    }

    angle_ = n["value"].as<double>();
    min_ = n["min"].as<double>();
    max_ = n["max"].as<double>();
}

