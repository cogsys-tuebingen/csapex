#ifndef SETTINGS_H
#define SETTINGS_H

/// PROJECT
#include <csapex/param/parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <string>
#include <csapex/utility/slim_signal.h>

namespace csapex
{

class SubgraphNode;

class CSAPEX_EXPORT Settings
{
public:
    static const std::string settings_file;
    static const std::string config_extension;
    static const std::string template_extension;
    static const std::string message_extension;
    static const std::string message_extension_compressed;
    static const std::string default_config;
    static const std::string config_selector;

    static const std::string namespace_separator;

    static std::string defaultConfigFile();
    static std::string defaultConfigPath();


public:
    static Settings NoSettings;

    Settings(bool load_from_config = true);

    bool isQuiet() const;
    void setQuiet(bool quiet);

    bool knows(const std::string& name) const;

    template <typename T>
    T get(const std::string& name) const
    {
        auto pos = settings_.find(name);
        if(pos == settings_.end()) {
            throw std::runtime_error(std::string("settings.get: unknown parameter '") + name + "'");
        }

        const Entry& entry = pos->second;
        return entry.parameter->as<T>();
    }

    template <typename T>
    T get(const std::string& name, T def)
    {
        auto pos = settings_.find(name);
        if(pos == settings_.end()) {
            param::ValueParameter::Ptr p(new param::ValueParameter(name, csapex::param::ParameterDescription()));
            p->set(def);
            addTemporary(p);
            settingsChanged(name);
            return def;
        }

        Entry& entry = pos->second;
        return entry.parameter->as<T>();
    }


    void add(csapex::param::Parameter::Ptr p, bool persistent);
    csapex::param::ParameterPtr get(const std::string& name);

    void addTemporary(csapex::param::Parameter::Ptr p);
    void addPersistent(csapex::param::Parameter::Ptr p);


    template <typename T>
    void set(const std::string& name, const T& val)
    {
        auto pos = settings_.find(name);
        if(pos == settings_.end()) {
            param::ValueParameter::Ptr p(new param::ValueParameter(name, csapex::param::ParameterDescription()));
            p->set(val);
            addTemporary(p);

        } else {
            Entry& entry = pos->second;
            entry.parameter->set<T>(val);
        }
        settingsChanged(name);
    }

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

#endif // SETTINGS_H
