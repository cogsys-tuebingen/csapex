/// HEADER
#include <csapex/core/settings.h>

/// PROJECT
#include <utils_param/io.h>

/// SYSTEM
#include <pwd.h>
#include <boost/filesystem.hpp>
#include <fstream>

using namespace csapex;

const std::string Settings::settings_file = defaultConfigPath() + "cfg/persistent_settings";
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

    load();
}

void Settings::save()
{

    YAML::Emitter yaml;
    yaml << YAML::BeginSeq;

    for(std::map<std::string, param::Parameter::Ptr>::iterator it = settings_.begin(); it != settings_.end(); ++it) {
        param::Parameter::Ptr p = it->second;
        p->write(yaml);
    }

    yaml << YAML::EndSeq;

    std::ofstream ofs(settings_file.c_str());
    ofs << yaml.c_str();
}

void Settings::load()
{
    std::ifstream ifs(settings_file.c_str());
    YAML::Parser parser(ifs);

    YAML::Node doc;

    if(!parser.GetNextDocument(doc) || doc.Type() != YAML::NodeType::Sequence) {
        std::cerr << "cannot read the settings" << std::endl;
        return;
    }

    for(std::size_t i = 0, n = doc.size(); i < n; ++i) {
        param::Parameter::Ptr p;
        doc[i] >> p;

        settings_[p->name()] = p;
    }
}

bool Settings::isProcessingAllowed() const
{
    return processing_allowed_;
}

void Settings::setProcessingAllowed(bool processing)
{
    if(processing != processing_allowed_) {
        processing_allowed_ = processing;

        settingsChanged();
    }
}

void Settings::setCurrentConfig(const std::string& filename)
{
    if(filename != current_config_) {
        current_config_ = filename;

        std::string dir = current_config_.substr(0, current_config_.find_last_of('/')+1);
        chdir(dir.c_str());

        settingsChanged();
    }
}

std::string Settings::getConfig() const
{
    return current_config_;
}

void Settings::add(param::Parameter::Ptr p)
{
    settings_[p->name()] = p;
    settingsChanged();
}

bool Settings::knows(const std::string &name) const
{
    return settings_.find(name) != settings_.end();
}
