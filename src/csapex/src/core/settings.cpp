/// HEADER
#include <csapex/core/settings.h>

/// SYSTEM
#include <pwd.h>
#include <boost/filesystem.hpp>

using namespace csapex;

const std::string Settings::config_extension = ".apex";
const std::string Settings::template_extension = ".apext";
const std::string Settings::default_config = Settings::defaultConfigFile();
const std::string Settings::config_selector = "Configs(*" + Settings::config_extension + ");;LegacyConfigs(*.vecfg)";

const std::string Settings::namespace_separator = ":/:";

const int Settings::activity_marker_max_lifetime_ = 10;

const unsigned Settings::timer_history_length_ = 15;


std::string Settings::defaultConfigPath()
{
    struct passwd *pw = getpwuid(getuid());
    return std::string(pw->pw_dir) + "/.csapex/";
}

std::string Settings::defaultConfigFile()
{
    std::string dir = Settings::defaultConfigPath();

    if(!boost::filesystem3::exists(dir)) {
        boost::filesystem3::create_directories(dir);
    }

    std::string file = dir + "default" + Settings::config_extension;

    if(!boost::filesystem3::exists(file)) {
        //        createDefaultConfig(file);
    }

    return file;
}

Settings::Settings()
    : processing_allowed_(false)
{
    current_config_ = defaultConfigFile();
}

bool Settings::isProcessingAllowed() const
{
    return processing_allowed_;
}

void Settings::setProcessingAllowed(bool processing)
{
    processing_allowed_ = processing;

    settingsChanged();
}

void Settings::setCurrentConfig(const std::string& filename)
{
    current_config_ = filename;

    std::string dir = current_config_.substr(0, current_config_.find_last_of('/')+1);
    chdir(dir.c_str());

    settingsChanged();
}

std::string Settings::getConfig() const
{
    return current_config_;
}

void Settings::add(param::Parameter::Ptr p)
{
    settings_[p->name()] = p;
}
