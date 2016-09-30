/// HEADER
#include <csapex/param/output_text_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <boost/any.hpp>

using namespace csapex;
using namespace param;

OutputTextParameter::OutputTextParameter()
    : Parameter("noname", ParameterDescription())
{
}

OutputTextParameter::OutputTextParameter(const std::string &name, const ParameterDescription& description)
    : Parameter(name, description)
{
}

OutputTextParameter::~OutputTextParameter()
{

}

std::string OutputTextParameter::toStringImpl() const
{
    std::stringstream v;
    v << "[text: " << text_ << "]";
    return v.str();
}

const std::type_info &OutputTextParameter::type() const
{
    Lock l = lock();
    return typeid(std::string);
}

void OutputTextParameter::get_unsafe(boost::any& out) const
{
    out = text_;
}
bool OutputTextParameter::set_unsafe(const boost::any& v)
{
    auto val = boost::any_cast<std::string>(v);
    if(val != text_) {
        text_ = val;
        return true;
    }

    return false;
}

void OutputTextParameter::doSerialize(YAML::Node& n) const
{
}
void OutputTextParameter::doDeserialize(const YAML::Node& n)
{
    if(!n["text"].IsDefined()) {
        return;
    }

    text_ = n["text"].as<std::string>();
}

void OutputTextParameter::doSetValueFrom(const Parameter& other)
{
    const OutputTextParameter* text = dynamic_cast<const OutputTextParameter*>(&other);
    if(text) {
        if(text_ != text->text_) {
            text_ = text->text_;
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void OutputTextParameter::doClone(const Parameter& other)
{
    const OutputTextParameter* text = dynamic_cast<const OutputTextParameter*>(&other);
    if(text) {
        text_ = text->text_;
    } else {
        throw std::runtime_error("bad clone, invalid types");
    }
}

