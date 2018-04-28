#ifndef SETTINGS_H
#define SETTINGS_H

/// PROJECT
#include <csapex/param/parameter.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/param/value_parameter.h>
#include <csapex/utility/slim_signal.h>
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <string>

namespace csapex
{

class CSAPEX_CORE_EXPORT Settings
{
public:
    static const std::string settings_file;
    static const std::string config_extension;
    static const std::string template_extension;
    static const std::string message_extension;
    static const std::string message_extension_compressed;
    static const std::string message_extension_binary;
    static const std::string default_config;
    static const std::string config_selector;

    static const std::string namespace_separator;

    static std::string defaultConfigFile();
    static std::string defaultConfigPath();


public:
    Settings();
    virtual ~Settings();

    bool isDirty() const;

    bool isQuiet() const;
    void setQuiet(bool quiet);

    virtual bool knows(const std::string& name) const = 0;

    virtual void add(csapex::param::Parameter::Ptr p, bool persistent) = 0;
    virtual csapex::param::ParameterPtr get(const std::string& name) const = 0;
    virtual csapex::param::ParameterPtr getNoThrow(const std::string& name) const = 0;

    void addTemporary(csapex::param::Parameter::Ptr p);
    void addPersistent(csapex::param::Parameter::Ptr p);

    virtual void savePersistent() = 0;
    virtual void loadPersistent() = 0;

    virtual void saveTemporary(YAML::Node& node) = 0;
    virtual void loadTemporary(YAML::Node& node) = 0;

    template <typename T>
    T get(const std::string& name) const
    {
        auto param = getNoThrow(name);
        if(!param) {
            throw std::runtime_error(std::string("settings.get: unknown parameter '") + name + "'");
        }

        return param->as<T>();
    }

    template <typename T>
    T get(const std::string& name, const T default_value) const
    {
        auto param = getNoThrow(name);
        if(!param) {
            return default_value;
        }

        return param->as<T>();
    }

    template <typename T>
    T getPersistent(const std::string& name, T def)
    {
        auto param = getNoThrow(name);
        if(!param) {
            param::ValueParameter::Ptr p(new param::ValueParameter(name, csapex::param::ParameterDescription()));
            p->set(def);
            addPersistent(p);
            return def;
        }

        return param->as<T>();
    }

    template <typename T>
    T getTemporary(const std::string& name, T def)
    {
        auto param = getNoThrow(name);
        if(!param) {
            param::ValueParameter::Ptr p(new param::ValueParameter(name, csapex::param::ParameterDescription()));
            p->set(def);
            addTemporary(p);
            return def;
        }

        return param->as<T>();
    }

    template <typename T>
    void set(const std::string& name, const T& val)
    {
        auto param = getNoThrow(name);
        if(!param) {
            param::ValueParameter::Ptr p(new param::ValueParameter(name, csapex::param::ParameterDescription()));
            p->set(val);
            addTemporary(p);

        } else {
            param->set<T>(val);
        }

        settingsChanged(name);
    }


    template <typename T>
    void setPersistent(const std::string& name, const T& val)
    {
        auto param = getNoThrow(name);
        if(!param) {
            param::ValueParameter::Ptr p(new param::ValueParameter(name, csapex::param::ParameterDescription()));
            p->set(val);
            addPersistent(p);

        } else {
            param->set<T>(val);
        }

        settingsChanged(name);
    }

protected:
    void setNotDirty();
    void settingsChanged(const std::string& key);

    virtual void triggerSettingsChanged();

public:
    csapex::slim_signal::Signal<void(const std::string&)> setting_changed;
    csapex::slim_signal::Signal<void()> settings_changed;

protected:
    bool dirty_;
    bool quiet_;

    std::vector<std::string> changes_;
};

}

#endif // SETTINGS_H
