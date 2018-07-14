/// HEADER
#include <csapex/nodes/sticky_note.h>

/// PROJECT
#include <csapex/param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex;

void Note::setup(NodeModifier& node_modifier)
{
}

void Note::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declareText("text", ""));
    parameters.addParameter(param::factory::declareValue<int>("w", 16));
    parameters.addParameter(param::factory::declareValue<int>("h", 16));
}

bool Note::isIsolated() const
{
    return true;
}
