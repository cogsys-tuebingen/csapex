/// HEADER
#include <csapex/core/settings/settings_local.h>

/// PROJECT
#include <csapex/param/io.h>
#include <csapex/utility/delegate.h>
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

SettingsLocal SettingsLocal::NoSettings(false);

SettingsLocal::SettingsLocal(bool load_from_config)
    : quiet_(false)
{
    if(load_from_config) {
        load();
    }
}

bool SettingsLocal::isQuiet() const
{
    return quiet_;
}

void SettingsLocal::setQuiet(bool quiet)
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

void SettingsLocal::settingsChanged(const std::string &key)
{
    if(quiet_) {
        changes_.push_back(key);

    } else {
        setting_changed(key);
        settings_changed();
    }
}


void SettingsLocal::save()
{
    YAML::Emitter yaml;
    yaml << YAML::BeginSeq;

    for(auto it = settings_.begin(); it != settings_.end(); ++it) {
        Entry& entry = it->second;
        if(entry.persistent) {
            csapex::param::Parameter::Ptr p = entry.parameter;
            YAML::Node n;
            p->serialize_yaml(n);
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

void SettingsLocal::load()
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
        p->setUUID(UUIDProvider::makeUUID_without_parent(std::string(":") + p->name()));

        Entry& entry = settings_[p->name()];
        entry.parameter = p;
        entry.persistent = true;
    }
}

void SettingsLocal::add(csapex::param::Parameter::Ptr p, bool persistent)
{
    p->setUUID(UUIDProvider::makeUUID_without_parent(std::string(":") + p->name()));

    Entry& entry = settings_[p->name()];
    entry.parameter = p;
    entry.persistent = persistent;

    observe(p->parameter_changed, [this](param::Parameter*) {
        settings_changed();
    });

    settingsChanged(p->name());
}
void SettingsLocal::addPersistent(csapex::param::Parameter::Ptr p)
{
    add(p, true);
}
void SettingsLocal::addTemporary(csapex::param::Parameter::Ptr p)
{
    add(p, false);
}

csapex::param::Parameter::Ptr SettingsLocal::get(const std::string &name) const
{
    const Entry& entry = settings_.at(name);
    return entry.parameter;
}
csapex::param::Parameter::Ptr SettingsLocal::getNoThrow(const std::string &name) const
{
    auto it = settings_.find(name);
    if(it != settings_.end()) {
        const Entry& entry = it->second;
        return entry.parameter;
    } else {
        return nullptr;
    }
}

bool SettingsLocal::knows(const std::string &name) const
{
    return settings_.find(name) != settings_.end();
}
