/// HEADER
#include <csapex/param/output_text_parameter.h>

/// PROJECT
#include <csapex/param/register_parameter.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <any>

CSAPEX_REGISTER_PARAM(OutputTextParameter)

using namespace csapex;
using namespace param;

OutputTextParameter::OutputTextParameter() : ParameterImplementation("noname", ParameterDescription())
{
}

OutputTextParameter::OutputTextParameter(const std::string& name, const ParameterDescription& description) : ParameterImplementation(name, description)
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

const std::type_info& OutputTextParameter::type() const
{
    Lock l = lock();
    return typeid(std::string);
}

void OutputTextParameter::get_unsafe(std::any& out) const
{
    out = text_;
}
bool OutputTextParameter::set_unsafe(const std::any& v)
{
    auto val = std::any_cast<std::string>(v);
    if (val != text_) {
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
    if (!n["text"].IsDefined()) {
        return;
    }

    text_ = n["text"].as<std::string>();
}

bool OutputTextParameter::cloneDataFrom(const Clonable& other)
{
    if (const OutputTextParameter* text = dynamic_cast<const OutputTextParameter*>(&other)) {
        bool value_changed = text_ != text->text_;
        *this = *text;
        if (value_changed) {
            triggerChange();
            return true;
        }
    } else if (const Parameter* param = dynamic_cast<const Parameter*>(&other)) {
        if (text_ != param->toString()) {
            text_ = param->toString();
            triggerChange();
        }
    }
    return false;
}

void OutputTextParameter::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Parameter::serialize(data, version);

    data << text_;
}

void OutputTextParameter::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Parameter::deserialize(data, version);

    data >> text_;
}
