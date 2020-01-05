/// HEADER
#include <csapex/core/settings/settings_impl.h>

/// PROJECT
#include <csapex/param/io.h>
#include <csapex/utility/delegate.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/utility/yaml.h>

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

SettingsImplementation SettingsImplementation::NoSettings(false);

SettingsImplementation::SettingsImplementation(bool load_from_config)
{
    if (load_from_config) {
        loadPersistent();
    }
}

void SettingsImplementation::savePersistent()
{
    YAML::Emitter yaml;
    yaml << YAML::BeginSeq;

    for (auto it = settings_.begin(); it != settings_.end(); ++it) {
        Entry& entry = it->second;
        if (entry.persistent) {
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

void SettingsImplementation::loadPersistent()
{
    if (!bf3::exists(settings_file)) {
        return;
    }

    YAML::Node doc = YAML::LoadFile(settings_file.c_str());

    if (doc.Type() != YAML::NodeType::Sequence) {
        std::cerr << "cannot read the settings" << std::endl;
        return;
    }

    for (std::size_t i = 0, n = doc.size(); i < n; ++i) {
        csapex::param::Parameter::Ptr p = doc[i].as<csapex::param::Parameter::Ptr>();
        p->setUUID(UUIDProvider::makeUUID_without_parent(std::string(":") + p->name()));

        Entry& entry = settings_[p->name()];
        entry.parameter = p;
        entry.persistent = true;
    }
}

void SettingsImplementation::saveTemporary(YAML::Node& node)
{
    YAML::Node yaml(YAML::NodeType::Sequence);

    for (auto it = settings_.begin(); it != settings_.end(); ++it) {
        Entry& entry = it->second;
        if (!entry.persistent) {
            YAML::Node n;
            csapex::param::Parameter::Ptr p = entry.parameter;
            p->serialize_yaml(n);

            yaml.push_back(n);
        }
    }

    setNotDirty();

    node["settings"] = yaml;
}

void SettingsImplementation::loadTemporary(YAML::Node& node)
{
    YAML::Node doc = node["settings"];
    if (doc.IsDefined()) {
        if (doc.Type() != YAML::NodeType::Sequence) {
            std::cerr << "cannot read the temporary settings" << std::endl;
            return;
        }

        for (std::size_t i = 0, n = doc.size(); i < n; ++i) {
            csapex::param::Parameter::Ptr p = doc[i].as<csapex::param::Parameter::Ptr>();
            p->setUUID(UUIDProvider::makeUUID_without_parent(std::string(":") + p->name()));

            if (knows(p->name())) {
                Entry& entry = settings_[p->name()];
                entry.parameter->cloneDataFrom(*p);
            } else {
                Entry& entry = settings_[p->name()];
                entry.parameter = p;
                entry.persistent = false;
            }
        }
    }

    dirty_ = false;
}

void SettingsImplementation::add(csapex::param::Parameter::Ptr p, bool persistent)
{
    p->setUUID(UUIDProvider::makeUUID_without_parent(std::string(":") + p->name()));

    Entry& entry = settings_[p->name()];
    entry.parameter = p;
    entry.persistent = persistent;

    observe(p->parameter_changed, [this](param::Parameter* p) { settingsChanged(p); });

    settingsChanged(p.get());
}

csapex::param::Parameter::Ptr SettingsImplementation::get(const std::string& name) const
{
    const Entry& entry = settings_.at(name);
    return entry.parameter;
}
csapex::param::Parameter::Ptr SettingsImplementation::getNoThrow(const std::string& name) const
{
    auto it = settings_.find(name);
    if (it != settings_.end()) {
        const Entry& entry = it->second;
        return entry.parameter;
    } else {
        return nullptr;
    }
}

bool SettingsImplementation::knows(const std::string& name) const
{
    return settings_.find(name) != settings_.end();
}
