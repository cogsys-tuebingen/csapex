/// HEADER
#include <csapex/param/color_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

using namespace csapex;
using namespace param;

ColorParameter::ColorParameter()
    : Parameter("color", ParameterDescription())
{
    def_.resize(3);
    colors_.resize(3);
}

ColorParameter::ColorParameter(const std::string &name, const ParameterDescription& description, int r, int g, int b)
    : Parameter(name, description)
{
    def_.resize(3);
    def_[0] = r;
    def_[1] = g;
    def_[2] = b;

    set(def_);
}

void ColorParameter::set(const std::vector<int> &v)
{
    colors_ = v;
    triggerChange();
}

const std::type_info& ColorParameter::type() const
{
    Lock l = lock();
    return typeid(colors_);
}

std::string ColorParameter::toStringImpl() const
{
    std::stringstream v;
    for(std::vector<int>::const_iterator c = colors_.begin(); c != colors_.end(); ++c) {
        v << std::hex << *c << std::dec << " ";
    }

    return std::string("[color: ") + v.str()  + "]";
}

void ColorParameter::get_unsafe(boost::any& out) const
{
    out = colors_;
}


bool ColorParameter::set_unsafe(const boost::any &v)
{
    auto col = boost::any_cast<std::vector<int> > (v);
    if(colors_  != col) {
        colors_  = col;
        return true;
    }

    return false;
}

std::vector<int> ColorParameter::def() const
{
    return def_;
}

std::vector<int> ColorParameter::value() const
{
    return colors_;
}


void ColorParameter::doSetValueFrom(const Parameter &other)
{
    const ColorParameter* color = dynamic_cast<const ColorParameter*>(&other);
    if(color) {
        if(colors_ != color->colors_) {
            colors_ = color->colors_;
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void ColorParameter::doClone(const Parameter &other)
{
    const ColorParameter* color = dynamic_cast<const ColorParameter*>(&other);
    if(color) {
        colors_ = color->colors_;
        def_ = color->def_;
    } else {
        throw std::runtime_error("bad clone, invalid types");
    }
}

void ColorParameter::doSerialize(YAML::Node& n) const
{
    n["values"] = colors_;
}

void ColorParameter::doDeserialize(const YAML::Node& n)
{
    if(n["values"].IsDefined()) {
        colors_ = n["values"].as< std::vector<int> >();
    }
}
