#ifndef SETTINGS_LOCAL_H
#define SETTINGS_LOCAL_H

/// COMPONENT
#include <csapex/core/settings.h>
#include <csapex/model/observer.h>

namespace csapex
{

class SettingsLocal : public Settings, public Observer
{
public:
    static SettingsLocal NoSettings;

public:
    SettingsLocal(bool load_from_config = true);

    bool isQuiet() const;
    void setQuiet(bool quiet);

    bool knows(const std::string& name) const;

    void add(csapex::param::Parameter::Ptr p, bool persistent);
    csapex::param::ParameterPtr get(const std::string& name) const;
    csapex::param::ParameterPtr getNoThrow(const std::string& name) const;

    void addTemporary(csapex::param::Parameter::Ptr p);
    void addPersistent(csapex::param::Parameter::Ptr p);

    void save();
    void load();

private:
    void settingsChanged(const std::string& key);

public:
    csapex::slim_signal::Signal<void(const std::string&)> setting_changed;
    csapex::slim_signal::Signal<void()> settings_changed;

    csapex::slim_signal::Signal<void (YAML::Node& e)> save_request;
    csapex::slim_signal::Signal<void (YAML::Node& n)> load_request;

    csapex::slim_signal::Signal<void (SubgraphNode*, YAML::Node& e)> save_detail_request;
    csapex::slim_signal::Signal<void (SubgraphNode*, YAML::Node& n)> load_detail_request;

private:
    struct Entry
    {
        csapex::param::Parameter::Ptr parameter;
        bool persistent;
    };

    std::map<std::string, Entry> settings_;

    bool quiet_;
    std::vector<std::string> changes_;
};

}

#endif // SETTINGS_LOCAL_H
