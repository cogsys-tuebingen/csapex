/// HEADER
#include <utils_param/trigger_parameter.h>

using namespace param;

TriggerParameter::TriggerParameter()
    : Parameter("noname")
{
}

TriggerParameter::TriggerParameter(const std::string &name)
    : Parameter(name)
{
}

TriggerParameter::~TriggerParameter()
{

}


const std::type_info& TriggerParameter::type() const
{
    return typeid(void);
}

std::string TriggerParameter::toStringImpl() const
{
    std::stringstream v;

    return std::string("[trigger]");
}

boost::any TriggerParameter::get_unsafe() const
{
    throw std::runtime_error("cannot read TriggerParameter");
}


void TriggerParameter::set_unsafe(const boost::any &v)
{
}


void TriggerParameter::doSetFrom(const Parameter &other)
{
}

void TriggerParameter::doWrite(YAML::Emitter& e) const
{
    e << YAML::Key << "type" << YAML::Value << "value";
}

void TriggerParameter::doRead(const YAML::Node& n)
{
    if(!n.FindValue("name")) {
        return;
    }

    n["name"] >> name_;

}

void TriggerParameter::trigger()
{
    triggerChange();
}
