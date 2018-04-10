/// HEADER
#include <csapex/param/output_text_parameter.h>

/// PROJECT
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <boost/any.hpp>

CSAPEX_REGISTER_PARAMETER_SERIALIZER(OutputTextParameter)

using namespace csapex;
using namespace param;

OutputTextParameter::OutputTextParameter()
    : ParameterImplementation("noname", ParameterDescription())
{
}

OutputTextParameter::OutputTextParameter(const std::string &name, const ParameterDescription& description)
    : ParameterImplementation(name, description)
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
        if(text_ != other.toString()) {
            text_ = other.toString();
            triggerChange();
        }
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



void OutputTextParameter::serialize(SerializationBuffer &data) const
{
    Parameter::serialize(data);

    data << text_;
}

void OutputTextParameter::deserialize(const SerializationBuffer& data)
{
    Parameter::deserialize(data);

    data >> text_;
}

