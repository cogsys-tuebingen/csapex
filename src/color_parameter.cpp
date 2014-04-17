/// HEADER
#include <utils_param/color_parameter.h>

using namespace param;

ColorParameter::ColorParameter()
    : Parameter("color")
{
    def_.resize(3);
    colors_.resize(3);
}

ColorParameter::ColorParameter(const std::string &name, int r, int g, int b)
    : Parameter(name)
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

boost::any ColorParameter::get_unsafe() const
{
    return colors_;
}


void ColorParameter::set_unsafe(const boost::any &v)
{
    colors_ = boost::any_cast<std::vector<int> > (v);
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
        colors_ = color->colors_;
        triggerChange();
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

void ColorParameter::doWrite(YAML::Emitter& e) const
{
    e << YAML::Key << "type" << YAML::Value << "color";
    e << YAML::Key << "values" << YAML::Value << colors_;
}

namespace {
template <typename T>
T __read(const YAML::Node& n) {
    T v;
    n >> v;
    return v;
}
}

void ColorParameter::doRead(const YAML::Node& n)
{
    if(!exists(n, "name")) {
        return;
    }

    n["name"] >> name_;

    if(exists(n, "values")) {
        n["values"] >> colors_;
    }
}
