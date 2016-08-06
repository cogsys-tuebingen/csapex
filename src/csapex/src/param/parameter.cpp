/// HEADER
#include <csapex/param/parameter.h>

/// PROJECT
#include <csapex/param/io.h>
#include <csapex/param/value_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <cxxabi.h>
#include <iostream>

using namespace csapex;
using namespace param;

Parameter::Parameter(const std::string &name, const ParameterDescription &description)
    : name_(name), description_(description), enabled_(true), temporary_(false), hidden_(false), interactive_(false)
{
}

Parameter::Parameter(const Parameter& other)
    : name_(other.name_), uuid_(other.uuid_),
      description_(other.description_), enabled_(other.enabled_), temporary_(other.temporary_), hidden_(other.hidden_), interactive_(other.interactive_),
      dict_(other.dict_)
{
}

Parameter::~Parameter()
{
    destroyed(this);
}

void Parameter::setUUID(const UUID& uuid)
{
    uuid_ = uuid;
}

UUID Parameter::getUUID() const
{
    return uuid_;
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


void Parameter::setHidden(bool hidden)
{
    hidden_ = hidden;
}

bool Parameter::isHidden() const
{
    return hidden_;
}


void Parameter::setTemporary(bool temporary)
{
    temporary_ = temporary;
}

bool Parameter::isTemporary() const
{
    return temporary_;
}

bool Parameter::hasState() const
{
    return true;
}

Parameter::Lock Parameter::lock() const
{
    return Lock(new std::unique_lock<std::recursive_mutex>(mutex_));
}

void Parameter::triggerChange()
{
    parameter_changed(this);
}

void Parameter::setName(const std::string &name)
{
    name_ = name;
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

const ParameterDescription& Parameter::description() const
{
    return description_;
}

void Parameter::setDescription(const ParameterDescription &desc)
{
    description_ = desc;
}

std::string Parameter::toStringImpl() const
{
    throw std::logic_error("not implemented");
}

void Parameter::serialize(YAML::Node &n) const
{
    n["type"] = TYPE();
    n["name"] = name();

    if(interactive_) {
        n["interactive"] = interactive_;
    }

    if(!dict_.empty()) {
        n["dict"] = dict_;
    }

    try {
        doSerialize(n);
    } catch(const std::exception& e) {
        std::cerr << "cannot serialize parameter " << name() << ": " << e.what() << std::endl;
        throw e;
    }
}

void Parameter::deserialize(const YAML::Node &n)
{
    if(n["interactive"].IsDefined()) {
        interactive_ = n["interactive"].as<bool>();
    }
    if(n["dict"].IsDefined()) {
        dict_ = n["dict"].as<std::map<std::string, param::ParameterPtr>>();
    }

    try {
        doDeserialize(n);
    } catch(const std::exception& e) {
        std::cerr << "cannot deserialize parameter " << name() << ": " << e.what() << std::endl;
        throw e;
    }
}

Parameter& Parameter::operator = (const Parameter& p)
{
    setValueFrom(p);
    return *this;
}

void Parameter::setValueFrom(const Parameter &other)
{
    name_ = other.name_;
    interactive_ = other.interactive_;
    enabled_ = other.enabled_;
    dict_ = other.dict_;
    doSetValueFrom(other);
}

void Parameter::clone(const Parameter &other)
{
    name_ = other.name_;
    description_ = other.description_;
    interactive_ = other.interactive_;
    enabled_ = other.enabled_;
    dict_ = other.dict_;
    doClone(other);
}

boost::any Parameter::access_unsafe(const Parameter &p) const
{
    return p.get_unsafe();
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

bool Parameter::accepts(const std::type_info& t) const
{
    return type() == t;
}

void Parameter::setDictionaryEntry(const std::string& key, const param::ParameterPtr& p)
{
    dict_[key] = p;
    dictionary_entry_changed(key);
}

param::ParameterPtr Parameter::getDictionaryEntry(const std::string &key) const
{
    return dict_.at(key);
}


namespace csapex {
namespace param {
template <typename T>
void Parameter::setDictionaryValue(const std::string& key, const T& value)
{
    param::ValueParameterPtr p = std::make_shared<param::ValueParameter>();
    p->set(value);
    setDictionaryEntry(key, p);
}

template void Parameter::setDictionaryValue<bool>(const std::string& key, const bool& value);
template void Parameter::setDictionaryValue<int>(const std::string& key, const int& value);
template void Parameter::setDictionaryValue<double>(const std::string& key, const double& value);
template void Parameter::setDictionaryValue<std::string>(const std::string& key, const std::string& value);
}
}
