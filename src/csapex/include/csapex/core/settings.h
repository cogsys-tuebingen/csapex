#ifndef SETTINGS_H
#define SETTINGS_H

/// PROJECT
#include <csapex/param/parameter.h>
#include <csapex/csapex_export.h>
#include <csapex/param/value_parameter.h>
#include <csapex/utility/slim_signal.h>

/// SYSTEM
#include <string>

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
    Settings();
    virtual ~Settings();

    virtual bool isQuiet() const = 0;
    virtual void setQuiet(bool quiet) = 0;

    virtual bool knows(const std::string& name) const = 0;

    virtual void add(csapex::param::Parameter::Ptr p, bool persistent) = 0;
    virtual csapex::param::ParameterPtr get(const std::string& name) const = 0;
    virtual csapex::param::ParameterPtr getNoThrow(const std::string& name) const = 0;

    virtual void addTemporary(csapex::param::Parameter::Ptr p) = 0;
    virtual void addPersistent(csapex::param::Parameter::Ptr p) = 0;

    virtual void save() = 0;
    virtual void load() = 0;



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
    T get(const std::string& name, T def)
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
    }

public:
    csapex::slim_signal::Signal<void(const std::string&)> setting_changed;
    csapex::slim_signal::Signal<void()> settings_changed;

    csapex::slim_signal::Signal<void (YAML::Node& e)> save_request;
    csapex::slim_signal::Signal<void (YAML::Node& n)> load_request;

    csapex::slim_signal::Signal<void (SubgraphNode*, YAML::Node& e)> save_detail_request;
    csapex::slim_signal::Signal<void (SubgraphNode*, YAML::Node& n)> load_detail_request;
};

}

#endif // SETTINGS_H
