#ifndef SETTINGS_H
#define SETTINGS_H

/// PROJECT
#include <utils_param/parameter.h>
#include <utils_param/value_parameter.h>

/// SYSTEM
#include <string>
#include <boost/signals2.hpp>

namespace csapex
{

class Settings
{
public:
    static const std::string settings_file;
    static const std::string config_extension;
    static const std::string template_extension;
    static const std::string message_extension;
    static const std::string default_config;
    static const std::string config_selector;

    static const std::string namespace_separator;

    static std::string defaultConfigFile();
    static std::string defaultConfigPath();


public:
    Settings();

    std::string getConfig() const;
    void setCurrentConfig(const std::string& filename);

    bool knows(const std::string& name) const;

    template <typename T>
    T get(const std::string& name) const
    {
        std::map<std::string, param::Parameter::Ptr>::const_iterator pos = settings_.find(name);
        if(pos == settings_.end()) {
            throw std::runtime_error(std::string("settings.get: unknown parameter '") + name + "'");
        }

        return pos->second->as<T>();
    }

    template <typename T>
    T get(const std::string& name, T def)
    {
        std::map<std::string, param::Parameter::Ptr>::const_iterator pos = settings_.find(name);
        if(pos == settings_.end()) {
            param::ValueParameter::Ptr p(new param::ValueParameter(name, param::ParameterDescription()));
            p->set(def);
            add(p);
            settingsChanged();
            return def;
        }

        return pos->second->as<T>();
    }

    void add(param::Parameter::Ptr p);

    template <typename T>
    void set(const std::string& name, const T& val)
    {
        std::map<std::string, param::Parameter::Ptr>::const_iterator pos = settings_.find(name);
        if(pos == settings_.end()) {
            param::ValueParameter::Ptr p(new param::ValueParameter(name, param::ParameterDescription()));
            p->set(val);
            add(p);

        } else {
            pos->second->set<T>(val);
        }
        settingsChanged();
    }

    void save();
    void load();

public:
    boost::signals2::signal<void()> settingsChanged;

private:
    std::string current_config_;

    std::map<std::string, param::Parameter::Ptr> settings_;
};

}

#endif // SETTINGS_H
