/// HEADER
#include <csapex/model/generic_state.h>

/// PROJECT
#include <csapex/param/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <qglobal.h>

using namespace csapex;

GenericState::GenericState()
    : silent_(false),
      parameter_set_changed(new boost::signals2::signal<void()>),
      parameter_added(new boost::signals2::signal<void(param::ParameterPtr)>),
      parameter_removed(new boost::signals2::signal<void(param::ParameterPtr)>)
{

}

void GenericState::setParentUUID(const UUID &parent_uuid)
{
    parent_uuid_ = parent_uuid;

    for(std::map<std::string, param::Parameter::Ptr>::const_iterator it = params.begin(); it != params.end(); ++it) {
        it->second->setUUID(parent_uuid_);
    }
}

GenericState::Ptr GenericState::clone() const
{
    GenericState::Ptr r(new GenericState(*this));
    return r;
}

void GenericState::writeYaml(YAML::Node& out) const {
    out["params"] = params;

    std::vector<std::string> persistent_v(persistent.begin(), persistent.end());
    out["persistent_params"] = persistent_v;
}

void GenericState::readYaml(const YAML::Node& node) {
    if(node["params"].IsDefined()) {
        params = node["params"].as<std::map<std::string, param::Parameter::Ptr> >();
    }
    if(node["persistent_params"].IsDefined()) {
        std::vector<std::string> persistent_v = node["persistent_params"].as<std::vector<std::string> >();
        persistent.clear();
        persistent.insert(persistent_v.begin(), persistent_v.end());
    }
}

void GenericState::initializePersistentParameters()
{
    for(const std::string& name : persistent) {
        (*parameter_added)(params[name]);
    }
}

void GenericState::addParameter(param::Parameter::Ptr param)
{
    param::Parameter::Ptr old_value;
    if(params.find(param->name()) != params.end()) {
        // already here, keep value!
        try {
            param->setValueFrom(*params[param->name()]);
        } catch(const std::exception& e) {
            std::cerr << "cannot use serialized value for " << param->name() << ": " << e.what() << std::endl;
        }
    }
    apex_assert_hard(param->name() != "noname");
    if(std::find(order.begin(), order.end(), param->name()) == order.end()) {
        order.push_back(param->name());
    }

    registerParameter(param);
}

void GenericState::removeParameter(param::ParameterPtr param)
{
    params.erase(param->name());

    order.erase(std::find(order.begin(), order.end(), param->name()));

    unregisterParameter(param);
}

void GenericState::setParameterSetSilence(bool silent)
{
    silent_ = silent;
}

void GenericState::removeTemporaryParameters()
{
    for(std::map<std::string,bool>::iterator it = temporary.begin(); it != temporary.end(); ++it) {
        std::string name(it->first);
        param::Parameter::Ptr p = getParameter(name);

//        params.erase(params.find(name));
        order.erase(std::find(order.begin(), order.end(), name));

        (*parameter_removed)(p);
    }

    temporary.clear();

    triggerParameterSetChanged();
}

void GenericState::triggerParameterSetChanged()
{
    if(!silent_) {
        (*parameter_set_changed)();
    }
}

void GenericState::addTemporaryParameter(const param::Parameter::Ptr &param)
{
    addParameter(param);
    temporary[param->name()] = true;
}

void GenericState::removeTemporaryParameter(const param::Parameter::Ptr &param)
{
    removeParameter(param);
    temporary.erase(param->name());
}


void GenericState::addPersistentParameter(const param::Parameter::Ptr &param)
{
    persistent.insert(param->name());

    registerParameter(param);
}

void GenericState::registerParameter(const param::Parameter::Ptr &param)
{
    params[param->name()] = param;

    param->setUUID(parent_uuid_.getFullName());

    (*parameter_added)(param);
    triggerParameterSetChanged();
}

void GenericState::unregisterParameter(const param::Parameter::Ptr &param)
{
    params.erase(param->name());

    (*parameter_removed)(param);
    triggerParameterSetChanged();
}

void GenericState::setFrom(const GenericState &rhs)
{
    persistent = rhs.persistent;

    for(std::map<std::string, param::Parameter::Ptr>::const_iterator it = rhs.params.begin(); it != rhs.params.end(); ++it) {
        param::Parameter::Ptr p = it->second;
        std::string name = p->name();
        if(params.find(name) != params.end()) {
            params[name]->setValueFrom(*p);
        } else {
            params[name] = param::ParameterFactory::clone(p);
        }
    }

    initializePersistentParameters();
}

param::Parameter &GenericState::operator [](const std::string& name) const
{
    try {
        return *params.at(name);
    } catch(const std::exception& e) {
        throw std::runtime_error("cannot get parameter '" + name + "', doesn't exist: " + e.what());
    }
}

param::Parameter::Ptr GenericState::getParameter(const std::string &name) const
{
    try {
        return params.at(name);
    } catch(const std::exception& e) {
        throw std::runtime_error("cannot get parameter '" + name + "', doesn't exist: " + e.what());
    }
}

std::vector<param::Parameter::Ptr> GenericState::getParameters() const
{
    std::vector<param::Parameter::Ptr> result;
    for(std::vector<std::string>::const_iterator n = order.begin(); n != order.end(); ++n) {
        result.push_back(params.at(*n));
    }
    for(const std::string& p : persistent) {
        result.push_back(params.at(p));
    }

    return result;
}

std::vector<param::Parameter::Ptr> GenericState::getTemporaryParameters() const
{
    std::vector<param::Parameter::Ptr> result;
    typedef std::pair<const std::string&,bool> PAIR;
    for(const PAIR& pair : temporary) {
        result.push_back(params.at(pair.first));
    }

    return result;
}

std::size_t GenericState::getParameterCount() const
{
    return params.size();
}

template <typename T>
T GenericState::readParameter(const std::string& name) const
{
    try {
        return getParameter(name)->as<T>();
    } catch(const std::out_of_range& e) {
        throw std::runtime_error(std::string("unknown parameter '") + name + "'");
    }
}

template bool GenericState::readParameter<bool>(const std::string& name) const;
template double GenericState::readParameter<double>(const std::string& name) const;
template int GenericState::readParameter<int>(const std::string& name) const;
template std::string GenericState::readParameter<std::string>(const std::string& name) const;
template std::pair<int,int> GenericState::readParameter<std::pair<int,int> >(const std::string& name) const;
template std::pair<double,double> GenericState::readParameter<std::pair<double,double> >(const std::string& name) const;
template std::vector<int> GenericState::readParameter<std::vector<int> >(const std::string& name) const;
template std::vector<double> GenericState::readParameter<std::vector<double> >(const std::string& name) const;
