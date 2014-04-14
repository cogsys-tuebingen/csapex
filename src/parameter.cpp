/// HEADER
#include <utils_param/parameter.h>

using namespace param;

Parameter::Parameter(const std::string &name)
    : name_(name), enabled_(true), interactive_(false)
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

void Parameter::setInteractive(bool interactive)
{
    if(interactive != interactive_) {
        interactive_ = interactive;

        interactive_changed(this, interactive_);
    }
}

bool Parameter::isInteractive() const
{
    return interactive_;
}

void Parameter::triggerChange()
{
    parameter_changed(this);
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
    e << YAML::BeginMap;
    e << YAML::Key << "name" << YAML::Value << name();

    if(interactive_) {
        e << YAML::Key << "interactive" << YAML::Value << interactive_;
    }
    doWrite(e);

    e << YAML::EndMap;
}

void Parameter::read(const YAML::Node &n)
{
    if(exists(n, "interactive")) {
        n["interactive"] >> interactive_;
    }

    doRead(n);
}

void Parameter::setFrom(const Parameter &other)
{
    interactive_ = other.interactive_;
    enabled_ = other.enabled_;
    doSetFrom(other);
}

boost::any Parameter::access_unsafe(const Parameter &p) const
{
    return p.get_unsafe();
}

boost::any Parameter::get_unsafe() const
{
    throw std::logic_error("not implemented");
}

void Parameter::set_unsafe(const boost::any &v)
{
    throw std::logic_error("not implemented");
}

std::string Parameter::type2string(const std::type_info &type)
{
    int status;
    return abi::__cxa_demangle(type.name(), 0, 0, &status);
}

void Parameter::throwTypeError(const std::type_info &a, const std::type_info &b, const std::string& prefix) const
{
    throw std::runtime_error(prefix + std::string("'") + name() + "' is not of type '" + type2string(a) + "' but '" + type2string(b) + "'");
}
