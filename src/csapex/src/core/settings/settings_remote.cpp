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

SettingsRemote::SettingsRemote(SessionPtr session)
    : session_(session)
{
}

void SettingsRemote::save()
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestTarget::Settings, CoreRequests::CoreRequestType::Save);
}

void SettingsRemote::load()
{
    session_->sendRequest<CoreRequests>(CoreRequests::CoreRequestTarget::Settings, CoreRequests::CoreRequestType::Load);
}

void SettingsRemote::add(csapex::param::Parameter::Ptr p, bool persistent)
{
    AUUID param_id(UUIDProvider::makeUUID_without_parent(std::string(":") + p->name()));
    boost::any value;
    p->get_unsafe(value);
    if(const auto& response = session_->sendRequest<AddParameter>(param_id, p->name(), p->description().toString(), value, persistent))
    {
        if(response->getParameter()) {
            std::cerr << "created parameter " << response->getParameter()->getUUID() << std::endl;

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
    if(const auto& response = session_->sendRequest<RequestParameter>(param_id)) {
        apex_assert_hard(response->getParameter());

        if(response->getParameter()->ID() == param::NullParameter::NUMERICAL_ID) {
            return nullptr;
        }

        std::cerr << response->getParameter()->getUUID() << std::endl;

        // make a new parameter, when it gets changed relay the change to the remote server
        param::ParameterPtr proxy = response->getParameter()->clone<param::Parameter>();

        createParameterProxy(name, proxy);

        return proxy;
    }

    return nullptr;
}

void SettingsRemote::createParameterProxy(const std::string &name, param::ParameterPtr proxy) const
{
    proxy->parameter_changed.connect([this](param::Parameter* param){
        // request to set the parameter
        boost::any raw;
        param->get_unsafe(raw);
        CommandPtr change = std::make_shared<command::UpdateParameter>(param->getUUID(), raw);
        session_->write(change);

        settingsChanged(param->name());
    });

    cache_[name] = proxy;
}

bool SettingsRemote::knows(const std::string &name) const
{
    auto res = getNoThrow(name);
    return res != nullptr;
}
