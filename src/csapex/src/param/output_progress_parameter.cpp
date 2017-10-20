/// HEADER
#include <csapex/param/output_progress_parameter.h>

/// PROJECT
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <boost/any.hpp>

CSAPEX_REGISTER_PARAMETER_SERIALIZER(OutputProgressParameter)

using namespace csapex;
using namespace param;

OutputProgressParameter::OutputProgressParameter()
    : ParameterImplementation("noname", ParameterDescription()), value(0), maximum(100)
{
}

OutputProgressParameter::OutputProgressParameter(const std::string &name, const ParameterDescription& description)
    : ParameterImplementation(name, description), value(0), maximum(100)
{
}

OutputProgressParameter::~OutputProgressParameter()
{

}

std::string OutputProgressParameter::toStringImpl() const
{
    std::stringstream v;

    return std::string("[progress]");
}

void OutputProgressParameter::get_unsafe(boost::any& out) const
{
    out = value;
}
bool OutputProgressParameter::set_unsafe(const boost::any& v)
{
    auto val = boost::any_cast<int>(v);
    if(val != value) {
        value = val;
        return true;
    }

    return false;
}

void OutputProgressParameter::doSerialize(YAML::Node& /*n*/) const
{

}
void OutputProgressParameter::doDeserialize(const YAML::Node& n)
{
}

void OutputProgressParameter::doSetValueFrom(const Parameter& other)
{
    const OutputProgressParameter* progress = dynamic_cast<const OutputProgressParameter*>(&other);
    if(progress) {
        if(value != progress->value || maximum != progress->maximum) {
            value = progress->value;
            maximum = progress->maximum;
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void OutputProgressParameter::doClone(const Parameter& other)
{
    const OutputProgressParameter* progress = dynamic_cast<const OutputProgressParameter*>(&other);
    if(progress) {
        value = progress->value;
        maximum = progress->maximum;
    } else {
        throw std::runtime_error("bad clone, invalid types");
    }
}


void OutputProgressParameter::setProgress(int progress, int m)
{
    if(value != progress) {
        value = progress;
        triggerChange();
    }

    if(maximum != m) {
        maximum = m;
        scope_changed(this);
    }
}

double OutputProgressParameter::getProgress() const
{
    return value;
}

double OutputProgressParameter::getProgressMaximum() const
{
    return maximum;
}


void OutputProgressParameter::serialize(SerializationBuffer &data) const
{
    Parameter::serialize(data);

    data << value;
    data << maximum;
}

void OutputProgressParameter::deserialize(const SerializationBuffer& data)
{
    Parameter::deserialize(data);

    data >> value;
    data >> maximum;
}
