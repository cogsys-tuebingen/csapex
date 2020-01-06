/// HEADER
#include <csapex/param/output_progress_parameter.h>

/// PROJECT
#include <csapex/param/register_parameter.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <any>

CSAPEX_REGISTER_PARAM(OutputProgressParameter)

using namespace csapex;
using namespace param;

OutputProgressParameter::OutputProgressParameter() : ParameterImplementation("noname", ParameterDescription()), value(0), maximum(100)
{
}

OutputProgressParameter::OutputProgressParameter(const std::string& name, const ParameterDescription& description) : ParameterImplementation(name, description), value(0), maximum(100)
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

void OutputProgressParameter::get_unsafe(std::any& out) const
{
    out = value;
}
bool OutputProgressParameter::set_unsafe(const std::any& v)
{
    auto val = std::any_cast<int>(v);
    if (val != value) {
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

bool OutputProgressParameter::cloneDataFrom(const Clonable& other)
{
    if (const OutputProgressParameter* progress = dynamic_cast<const OutputProgressParameter*>(&other)) {
        bool value_changed = value != progress->value || maximum != progress->maximum;
        *this = *progress;
        if (value_changed) {
            triggerChange();
        }
        return true;

    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }

    return false;
}

void OutputProgressParameter::advanceProgress(int step)
{
    setProgress(value + step, maximum);
}

void OutputProgressParameter::setProgress(int progress, int m)
{
    if (value != progress) {
        value = progress;
        triggerChange();
    }

    if (maximum != m) {
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

void OutputProgressParameter::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Parameter::serialize(data, version);

    data << value;
    data << maximum;
}

void OutputProgressParameter::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Parameter::deserialize(data, version);

    data >> value;
    data >> maximum;
}
