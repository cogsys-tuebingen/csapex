/// HEADER
#include <csapex/model/generic_state.h>

/// PROJECT
#include <utils_param/io.h>

using namespace csapex;

GenericState::GenericState()
    : silent_(false), parameter_set_changed(new boost::signals2::signal<void()>)
{

}

GenericState::Ptr GenericState::clone() const
{
    GenericState::Ptr r(new GenericState(*this));
    return r;
}

void GenericState::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "params" << YAML::Value << params;
}

void GenericState::readYaml(const YAML::Node& node) {
    if(exists(node, "params")) {
        node["params"] >> params;
    }
}

void GenericState::addParameter(param::Parameter::Ptr param)
{
    assert(param->name() != "noname");
    assert(std::find(order.begin(), order.end(), param->name()) == order.end());

    params[param->name()] = param;
    order.push_back(param->name());

    triggerParameterSetChanged();
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

        params.erase(params.find(name));
        order.erase(std::find(order.begin(), order.end(), name));
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

void GenericState::setFrom(const GenericState &rhs)
{
//    std::map<std::string, param::Parameter::Ptr> old_params = params;
//    std::vector<std::string> old_order = order;

//    *this = rhs;

//    params = old_params;
//    order = old_order;
//    order = old_order;

    for(std::map<std::string, param::Parameter::Ptr>::const_iterator it = rhs.params.begin(); it != rhs.params.end(); ++it) {
        param::Parameter::Ptr p = it->second;
        if(params.find(p->name()) != params.end()) {
            params[p->name()]->setValueFrom(*p);
        }
    }
}

param::Parameter &GenericState::operator [](const std::string& name) const
{
    return *params.at(name);
}

param::Parameter::Ptr GenericState::getParameter(const std::string &name) const
{
    return params.at(name);
}

std::vector<param::Parameter::Ptr> GenericState::getParameters() const
{
    std::vector<param::Parameter::Ptr> result;
    for(std::vector<std::string>::const_iterator n = order.begin(); n != order.end(); ++n) {
        result.push_back(params.at(*n));
    }
    return result;
}
