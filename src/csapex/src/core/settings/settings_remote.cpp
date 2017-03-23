/// HEADER
#include <csapex/core/settings/settings_remote.h>

/// PROJECT
#include <csapex/param/io.h>
#include <csapex/command/update_parameter.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/request_parameter.h>
#include <csapex/utility/uuid_provider.h>

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

bool SettingsRemote::isQuiet() const
{
    return false;
}

void SettingsRemote::setQuiet(bool quiet)
{
}

void SettingsRemote::save()
{
}

void SettingsRemote::load()
{
}

void SettingsRemote::add(csapex::param::Parameter::Ptr p, bool persistent)
{
}
void SettingsRemote::addPersistent(csapex::param::Parameter::Ptr p)
{
}
void SettingsRemote::addTemporary(csapex::param::Parameter::Ptr p)
{
}

csapex::param::Parameter::Ptr SettingsRemote::get(const std::string &name) const
{
    auto it = cache_.find(name);
    if(it != cache_.end()) {
        return it->second;
    }

    AUUID param_id(UUIDProvider::makeUUID_without_parent(std::string(":") + name));
    std::shared_ptr<RequestParameter::ParameterRequest> request = std::make_shared<RequestParameter::ParameterRequest>(param_id);

    ResponseConstPtr raw_response = session_->sendRequest(request);
    if(const std::shared_ptr<RequestParameter::ParameterResponse const>& response = std::dynamic_pointer_cast<RequestParameter::ParameterResponse const>(raw_response)) {
        apex_assert_hard(response->getParameter());

        std::cerr << response->getParameter()->getUUID() << std::endl;

        // TODO: make a new parameter, when it gets changed relay the change to the remote server
        param::ParameterPtr proxy = response->getParameter()->clone<param::Parameter>();

        proxy->parameter_changed.connect([this](param::Parameter* param){
            // request to set the parameter
            boost::any raw;
            param->get_unsafe(raw);
            CommandPtr change = std::make_shared<command::UpdateParameter>(param->getUUID(), raw);
            session_->write(change);
        });

        cache_[name] = proxy;

        return proxy;
    }

    throw std::out_of_range(std::string("parameter ") + name + " does not exist.");
}
csapex::param::Parameter::Ptr SettingsRemote::getNoThrow(const std::string &name) const
{
    return nullptr;
}

bool SettingsRemote::knows(const std::string &name) const
{
    return false;
}
