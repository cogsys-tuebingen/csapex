/// HEADER
#include <csapex/model/generic_state.h>

/// PROJECT
#include <csapex/param/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <qglobal.h>
#include <boost/regex.hpp>

using namespace csapex;

GenericState::GenericState()
    : silent_(false),
      parameter_set_changed(new slim_signal::Signal<void()>),
      parameter_added(new slim_signal::Signal<void(csapex::param::ParameterPtr)>),
      parameter_removed(new slim_signal::Signal<void(csapex::param::ParameterPtr)>)
{

}

void GenericState::setParentUUID(const UUID &parent_uuid)
{
    apex_assert_hard(!parent_uuid.composite());

    parent_uuid_ = parent_uuid;

    for(std::map<std::string, csapex::param::Parameter::Ptr>::const_iterator it = params.begin(); it != params.end(); ++it) {
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
        params = node["params"].as<std::map<std::string, csapex::param::Parameter::Ptr> >();
        for(auto pair : params) {
            apex_assert_hard(pair.first == pair.second->name());
            legacy.insert(pair.first);
        }
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

void GenericState::addParameter(csapex::param::Parameter::Ptr param)
{
    std::string param_name = param->name();

    apex_assert_hard(param_name != "noname");
    auto legacy_pos = legacy.find(param_name);
    auto param_pos = params.find(param_name);
    if(param_pos != params.end()) {
        if(legacy_pos == legacy.end()) {
            throw std::logic_error(std::string("a parameter with the name ") + param_name + " has already been added.");
        }
        *param = *param_pos->second;
    }
    registerParameter(param);

    if(legacy_pos != legacy.end()) {
        legacy.erase(legacy_pos);
    }
    if(std::find(order.begin(), order.end(), param_name) == order.end()) {
        order.push_back(param_name);
    }    

    std::string valid_name = param_name;

    boost::regex to_remove("~");
    valid_name = boost::regex_replace(valid_name, to_remove, std::string(""));

    // generate a valid name, valid characters are a-z, A-Z, 0-9, / and _.
    boost::regex invalid("[^0-9a-zA-Z/_]");
    if (boost::regex_search(valid_name, invalid)) {
        valid_name = boost::regex_replace(valid_name, invalid, std::string("_"));
    }

    param_valid_name_cache[valid_name] = param_name;
}

void GenericState::removeParameter(csapex::param::ParameterPtr param)
{
    params.erase(param->name());

    auto pos = std::find(order.begin(), order.end(), param->name());
    if(pos != order.end()) {
        order.erase(pos);
    }

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
        csapex::param::Parameter::Ptr p = getParameter(name);

        // don't erase the param itself, remember the value for future!
        // don't -> params.erase(params.find(name));
        order.erase(std::find(order.begin(), order.end(), name));

        (*parameter_removed)(p);
    }

    triggerParameterSetChanged();
}

void GenericState::triggerParameterSetChanged()
{
    if(!silent_) {
        (*parameter_set_changed)();
    }
}

void GenericState::addTemporaryParameter(const csapex::param::Parameter::Ptr &param)
{
    param->setTemporary(true);

    // check if there is an old version of the parameter
    csapex::param::Parameter::Ptr entry = param;
    std::string name = param->name();
    auto param_pos = params.find(name);
    if(param_pos != params.end()) {
        // the existing parameter should be temporary or legacy
        if(temporary.find(name) == temporary.end() && legacy.find(name) == legacy.end()) {
            throw std::runtime_error("trying to add a temporary parameter with the name "
                                     "of an existing parameter '" + name + "'");
        }
        param::Parameter::Ptr p = param_pos->second;
        *entry = *p;
        removeParameter(p);
    }
    temporary[entry->name()] = true;
    addParameter(entry);
}

void GenericState::removeTemporaryParameter(const csapex::param::Parameter::Ptr &param)
{
    removeParameter(param);
}


void GenericState::addPersistentParameter(const csapex::param::Parameter::Ptr &param)
{
    persistent.insert(param->name());

    registerParameter(param);
}

void GenericState::registerParameter(const csapex::param::Parameter::Ptr &param)
{
    params[param->name()] = param;

    param->setUUID(UUIDProvider::makeTypedUUID_forced(parent_uuid_, "param", param->name()));

    (*parameter_added)(param);
    triggerParameterSetChanged();
}

void GenericState::unregisterParameter(const csapex::param::Parameter::Ptr &param)
{
    params.erase(param->name());

    (*parameter_removed)(param);
    triggerParameterSetChanged();
}

void GenericState::setFrom(const GenericState &rhs)
{
    persistent = rhs.persistent;

    for(std::map<std::string, csapex::param::Parameter::Ptr>::const_iterator it = rhs.params.begin(); it != rhs.params.end(); ++it) {
        csapex::param::Parameter::Ptr p = it->second;
        std::string name = p->name();
        if(params.find(name) != params.end()) {
            params[name]->setValueFrom(*p);
        } else {
            params[name] = csapex::param::ParameterFactory::clone(p);
            legacy.insert(name);
        }
    }

    initializePersistentParameters();
}

csapex::param::Parameter &GenericState::operator [](const std::string& name) const
{
    try {
        return *params.at(name);
    } catch(const std::exception& e) {
        throw std::runtime_error("cannot get parameter '" + name + "', doesn't exist: " + e.what());
    }
}

csapex::param::Parameter::Ptr GenericState::getParameter(const std::string &name) const
{
    try {
        return params.at(name);
    } catch(const std::exception& e) {
        throw std::runtime_error("cannot get parameter '" + name + "', doesn't exist: " + e.what());
    }
}
csapex::param::Parameter::Ptr GenericState::getMappedParameter(const std::string &name) const
{
    try {
        auto pos = param_valid_name_cache.find(name);
        return params.at(pos->second);
    } catch(const std::exception& e) {
        throw std::runtime_error("cannot get parameter '" + name + "', doesn't exist: " + e.what());
    }
}

std::vector<csapex::param::Parameter::Ptr> GenericState::getParameters() const
{
    std::vector<csapex::param::Parameter::Ptr> result;
    for(std::vector<std::string>::const_iterator n = order.begin(); n != order.end(); ++n) {
        result.push_back(params.at(*n));
    }
    for(const std::string& p : persistent) {
        result.push_back(params.at(p));
    }

    return result;
}

std::vector<csapex::param::Parameter::Ptr> GenericState::getTemporaryParameters() const
{
    std::vector<csapex::param::Parameter::Ptr> result;
    for(const auto& pair : temporary) {
        auto pos = params.find(pair.first);
        if(pos != params.end()) {
            result.push_back(pos->second);
        }
    }

    return result;
}

std::vector<csapex::param::Parameter::Ptr> GenericState::getPersistentParameters() const
{
    std::vector<csapex::param::Parameter::Ptr> result;
    for(const auto& name : persistent) {
        auto pos = params.find(name);
        if(pos != params.end()) {
            result.push_back(pos->second);
        }
    }

    return result;
}

std::size_t GenericState::getParameterCount() const
{
    return params.size();
}

bool GenericState::hasParameter(const std::string& name) const
{
    auto pos = param_valid_name_cache.find(name);
    if(pos == param_valid_name_cache.end()) {
        return false;
    }

    return params.find(pos->second) != params.end();
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

template CSAPEX_EXPORT bool GenericState::readParameter<bool>(const std::string& name) const;
template CSAPEX_EXPORT double GenericState::readParameter<double>(const std::string& name) const;
template CSAPEX_EXPORT int GenericState::readParameter<int>(const std::string& name) const;
template CSAPEX_EXPORT std::string GenericState::readParameter<std::string>(const std::string& name) const;
template CSAPEX_EXPORT std::pair<int,int> GenericState::readParameter<std::pair<int,int> >(const std::string& name) const;
template CSAPEX_EXPORT std::pair<double,double> GenericState::readParameter<std::pair<double,double> >(const std::string& name) const;
template CSAPEX_EXPORT std::vector<int> GenericState::readParameter<std::vector<int> >(const std::string& name) const;
template CSAPEX_EXPORT std::vector<double> GenericState::readParameter<std::vector<double> >(const std::string& name) const;
