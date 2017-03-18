/// HEADER
#include <csapex/core/settings.h>

/// PROJECT
#include <csapex/param/io.h>
#include <csapex/utility/yaml_node_builder.h>

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

Settings Settings::NoSettings(false);

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

Settings::Settings(bool load_from_config)
    : quiet_(false)
{
    if(load_from_config) {
        load();
    }
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


void Settings::save()
{
    YAML::Emitter yaml;
    yaml << YAML::BeginSeq;

    for(auto it = settings_.begin(); it != settings_.end(); ++it) {
        Entry& entry = it->second;
        if(entry.persistent) {
            csapex::param::Parameter::Ptr p = entry.parameter;
            YAML::Node n;
            p->serialize(n);
            yaml << n;
        }
    }

    yaml << YAML::EndSeq;

#if WIN32
    uint32_t pid = GetCurrentProcessId();
#else
    uint32_t pid = getpid();
#endif

    bf3::path tmp_file = settings_file + "." + std::to_string(pid) + ".tmp";
    bf3::create_directories(tmp_file.parent_path());

    std::ofstream ofs(tmp_file.c_str());
    ofs << yaml.c_str();

    bf3::rename(tmp_file, settings_file);

    bf3::remove(tmp_file);
}

void Settings::load()
{
    if(!bf3::exists(settings_file)) {
        return;
    }

    YAML::Node doc = YAML::LoadFile(settings_file.c_str());

    if(doc.Type() != YAML::NodeType::Sequence) {
        std::cerr << "cannot read the settings" << std::endl;
        return;
    }

    for(std::size_t i = 0, n = doc.size(); i < n; ++i) {
        csapex::param::Parameter::Ptr p = doc[i].as<csapex::param::Parameter::Ptr>();

        Entry& entry = settings_[p->name()];
        entry.parameter = p;
        entry.persistent = true;
    }
}

void Settings::add(csapex::param::Parameter::Ptr p, bool persistent)
{
    Entry& entry = settings_[p->name()];
    entry.parameter = p;
    entry.persistent = persistent;

    settingsChanged(p->name());
}
void Settings::addPersistent(csapex::param::Parameter::Ptr p)
{
    add(p, true);
}
void Settings::addTemporary(csapex::param::Parameter::Ptr p)
{
    add(p, false);
}

csapex::param::Parameter::Ptr Settings::get(const std::string &name)
{
    const Entry& entry = settings_.at(name);
    return entry.parameter;
}

bool Settings::knows(const std::string &name) const
{
    return settings_.find(name) != settings_.end();
}
