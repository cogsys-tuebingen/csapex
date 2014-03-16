/// HEADER
#include <utils_param/null_parameter.h>

using namespace param;

NullParameter::NullParameter()
    : Parameter("null")
{
}


NullParameter::NullParameter(const std::string &name)
    : Parameter(name)
{
}

NullParameter::~NullParameter()
{

}

const std::type_info& NullParameter::type() const
{
    return typeid(void);
}

std::string NullParameter::toStringImpl() const
{
    return std::string("[null]");
}

boost::any NullParameter::get_unsafe() const
{
    throw std::runtime_error("cannot use null parameters");
}


void NullParameter::set_unsafe(const boost::any &v)
{
    throw std::runtime_error("cannot use null parameters");
}


void NullParameter::setFrom(const Parameter &other)
{
    throw std::runtime_error("cannot use null parameters");
}

void NullParameter::write(YAML::Emitter& e) const
{
}

void NullParameter::read(const YAML::Node& n)
{
}

