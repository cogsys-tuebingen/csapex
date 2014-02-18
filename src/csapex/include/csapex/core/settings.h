#ifndef SETTINGS_H
#define SETTINGS_H

/// PROJECT
#include <utils_param/parameter.h>

/// SYSTEM
#include <string>
#include <boost/signals2.hpp>

namespace csapex
{

class Settings
{
public:
    static const std::string config_extension;
    static const std::string template_extension;
    static const std::string default_config;
    static const std::string config_selector;

    static std::string defaultConfigFile();
    static std::string defaultConfigPath();


public:
    Settings();

    std::string getConfig() const;
    void setCurrentConfig(const std::string& filename);

    bool isProcessingAllowed() const;
    void setProcessingAllowed(bool processing);

    template <typename T>
    T get(const std::string& name) const
    {
        try {
            return settings_.at(name)->as<T>();
        } catch(const std::out_of_range& e) {
            throw std::runtime_error(std::string("settings.get: unknown parameter '") + name + "'");
        }
    }


    template <typename T>
    void set(const std::string& name, const T& val) const
    {
        try {
            settings_.at(name)->set<T>(val);
            settingsChanged();

        } catch(const std::out_of_range& e) {
            throw std::runtime_error(std::string("settings.set: unknown parameter '") + name + "'");
        }
    }



public:
    boost::signals2::signal<void()> settingsChanged;

private:
    std::string current_config_;
    bool processing_allowed_;

    std::map<std::string, param::Parameter::Ptr> settings_;
};

}

#endif // SETTINGS_H
