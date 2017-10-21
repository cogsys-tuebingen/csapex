/// HEADER
#include <csapex/core/settings/settings_remote.h>

/// PROJECT
#include <csapex/param/io.h>
#include <csapex/command/update_parameter.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/request_parameter.h>
#include <csapex/io/protcol/add_parameter.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/io/protcol/core_requests.h>
#include <csapex/param/null_parameter.h>
#include <csapex/io/protcol/parameter_changed.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

#ifdef WIN32
#include <windows.h>
#include <Shlobj.h>
#else
#include <pwd.h>
#include <unistd.h>
#endif

#if (BOOST_VERSION / 100000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 54
namespace bf3 = boost::filesystem;
#else
namespace bf3 = boost::filesystem3;
#endif


using namespace csapex;

SettingsRemote::SettingsRemote(Session& session)
    : Remote(session)
{
}

void SettingsRemote::savePersistent()
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::SettingsSavePersistent);
}

void SettingsRemote::loadPersistent()
{
    session_.sendRequest<CoreRequests>(CoreRequests::CoreRequestType::SettingsLoadPersistent);
}

void SettingsRemote::saveTemporary(YAML::Node& node)
{
    throw std::runtime_error("saving temporary settings not implemented for remote settings");
}

void SettingsRemote::loadTemporary(YAML::Node& node)
{
    throw std::runtime_error("loading temporary settings not implemented for remote settings");
}

void SettingsRemote::add(csapex::param::Parameter::Ptr p, bool persistent)
{
    AUUID param_id(UUIDProvider::makeUUID_without_parent(std::string(":") + p->name()));
    boost::any value;
    p->get_unsafe(value);
    if(const auto& response = session_.sendRequest<AddParameter>(param_id, p->name(), p->description().toString(), value, persistent))
    {
        if(response->getParameter()) {
            //std::cerr << "created parameter " << response->getParameter()->getUUID() << std::endl;

            // make a new parameter, when it gets changed relay the change to the remote server
            param::ParameterPtr proxy = response->getParameter()->clone<param::Parameter>();

            createParameterProxy(p->name(), proxy);
        }
    }
}

csapex::param::Parameter::Ptr SettingsRemote::get(const std::string &name) const
{
    auto res = getNoThrow(name);
    if(!res) {
        throw std::out_of_range(std::string("parameter ") + name + " does not exist.");
    }

    return res;
}
csapex::param::Parameter::Ptr SettingsRemote::getNoThrow(const std::string &name) const
{
    auto it = cache_.find(name);
    if(it != cache_.end()) {
        return it->second;
    }

    AUUID param_id(UUIDProvider::makeUUID_without_parent(std::string(":") + name));
    if(const auto& response = session_.sendRequest<RequestParameter>(param_id)) {
        apex_assert_hard(response->getParameter());

        if(response->getParameter()->ID() == param::NullParameter::NUMERICAL_ID) {
            return nullptr;
        }

        //std::cerr << response->getParameter()->getUUID() << std::endl;

        // make a new parameter, when it gets changed relay the change to the remote server
        param::ParameterPtr proxy = response->getParameter()->clone<param::Parameter>();

        createParameterProxy(name, proxy);

        return proxy;
    }

    return nullptr;
}

void SettingsRemote::createParameterProxy(const std::string &name, param::ParameterPtr proxy) const
{
    SettingsRemote* self = const_cast<SettingsRemote*>(this);
    proxy->parameter_changed.connect([self](param::Parameter* param){
        // request to set the parameter
        boost::any raw;
        param->get_unsafe(raw);
        CommandPtr change = std::make_shared<command::UpdateParameter>(param->getUUID(), raw);
        self->session_.write(change);

        self->settingsChanged(param->name());
    });

    cache_[name] = proxy;
}

bool SettingsRemote::knows(const std::string &name) const
{
    auto res = getNoThrow(name);
    return res != nullptr;
}


void SettingsRemote::handleBroadcast(const BroadcastMessageConstPtr& message)
{
    if(auto parameter_change = std::dynamic_pointer_cast<ParameterChanged const>(message)) {
        if(parameter_change->getUUID().global()) {
            auto pos = cache_.find(parameter_change->getUUID().globalName());
            if(pos != cache_.end()) {
                param::ParameterPtr p = pos->second;
                if(p->set_unsafe(parameter_change->getValue())) {
                    p->triggerChange();
                    std::cerr << "setting " << parameter_change->getUUID().globalName() << " changed to " << p->toString() << std::endl;
                }
            }
        }
    }
}
