/// HEADER
#include <utils_param/parameter.h>

using namespace param;

Parameter::Parameter(const std::string &name)
    : name_(name), enabled_(true)
{
}


Parameter::~Parameter()
{

}

void Parameter::setEnabled(bool enabled)
{
    if(enabled != enabled_) {
        enabled_ = enabled;

        parameter_enabled(this, enabled);
    }
}

bool Parameter::isEnabled() const
{
    return enabled_;
}

std::string Parameter::name() const
{
    return name_;
}

const std::type_info& Parameter::type() const
{
    return typeid(void);
}

std::string Parameter::toString() const
{
    return std::string("[") + name() + ": " + toStringImpl() + "]";
}

std::string Parameter::toStringImpl() const
{
    throw std::logic_error("not implemented");
}

void Parameter::write(YAML::Emitter &e) const
{
    throw std::logic_error("not implemented");
}

void Parameter::read(const YAML::Node &n)
{
    throw std::logic_error("not implemented");
}

void Parameter::setFrom(const Parameter &other)
{
    throw std::logic_error("not implemented");
}

Parameter::Ptr Parameter::empty()
{
    return Parameter::Ptr(new Parameter("loading"));
}

boost::any Parameter::get_unsafe() const
{
    throw std::logic_error("not implemented");
}

void Parameter::set_unsafe(const boost::any &v)
{
    throw std::logic_error("not implemented");
}

std::string Parameter::type2string(const std::type_info &type) const
{
    int status;
    return abi::__cxa_demangle(type.name(), 0, 0, &status);
}

void Parameter::throwTypeError(const std::type_info &a, const std::type_info &b, const std::string& prefix) const
{
    throw std::runtime_error(prefix + std::string("this is not of type '") + type2string(a) + "' but '" + type2string(b) + "'");
}
