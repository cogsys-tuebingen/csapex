/// HEADER
#include <csapex/core/settings.h>

/// PROJECT
#include <csapex/param/io.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#include <fstream>
#include <iostream>
#include <string>

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

const std::string Settings::settings_file = defaultConfigPath() + "cfg/persistent_settings";
const std::string Settings::config_extension = ".apex";
const std::string Settings::template_extension = ".apexs";
const std::string Settings::message_extension = ".apexm";
const std::string Settings::message_extension_compressed = ".apexm.gz";
const std::string Settings::default_config = Settings::defaultConfigFile();
const std::string Settings::config_selector = "Configs(*" + Settings::config_extension + ");;LegacyConfigs(*.vecfg)";

const std::string Settings::namespace_separator = ":/:";


std::string Settings::defaultConfigPath()
{
#ifdef WIN32
	CHAR path[MAX_PATH];
	if (SUCCEEDED(SHGetFolderPath(NULL, CSIDL_PROFILE, NULL, 0, path))) {
		return std::string(path) + "/.csapex/";
	}
#else
    struct passwd *pw = getpwuid(getuid());
    return std::string(pw->pw_dir) + "/.csapex/";
#endif
}

std::string Settings::defaultConfigFile()
{
    std::string dir = Settings::defaultConfigPath();

    if(!bf3::exists(dir)) {
        bf3::create_directories(dir);
    }

    std::string file = dir + "default" + Settings::config_extension;

    if(!bf3::exists(file)) {
        //        createDefaultConfig(file);
    }

    return file;
}

Settings::Settings()
    : quiet_(false)
{
}

Settings::~Settings()
{

}

void Settings::addPersistent(csapex::param::Parameter::Ptr p)
{
    add(p, true);
}
void Settings::addTemporary(csapex::param::Parameter::Ptr p)
{
    add(p, false);
}


bool Settings::isQuiet() const
{
    return quiet_;
}

void Settings::setQuiet(bool quiet)
{
    if(quiet != quiet_) {
        quiet_ = quiet;

        if(!quiet && !changes_.empty()) {
            for(const std::string& key : changes_) {
                setting_changed(key);
            }
            changes_.clear();
            settings_changed();
        }
    }
}

void Settings::settingsChanged(const std::string &key)
{
    if(quiet_) {
        changes_.push_back(key);

    } else {
        setting_changed(key);
        settings_changed();
    }
}
