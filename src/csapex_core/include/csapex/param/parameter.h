#ifndef PARAMETER_H
#define PARAMETER_H

/// COMPONENT
#include <csapex/param/param_fwd.h>
#include <csapex/param/parameter_description.h>
#include <csapex/utility/uuid.h>
#include <csapex_param_export.h>
#include <csapex/serialization/streamable.h>

/// SYSTEM
#include <memory>
#include <csapex/utility/slim_signal.hpp>
#include <mutex>

/// FORWARD DECLARATIONS
namespace YAML {
class Node;
}

namespace boost
{
class any;
}

namespace csapex {
namespace param {

class CSAPEX_PARAM_EXPORT Parameter : public Streamable
{
public:
    friend class ParameterFactory;
    friend class ParameterBuilder;

    typedef std::shared_ptr<Parameter> Ptr;

    typedef std::shared_ptr<std::unique_lock<std::recursive_mutex>> Lock;

public:
    slim_signal::Signal<void(Parameter*)> parameter_changed;
    slim_signal::Signal<void(Parameter*)> scope_changed;
    slim_signal::Signal<void(Parameter*, bool)> interactive_changed;
    slim_signal::Signal<void(Parameter*, bool)> parameter_enabled;
    slim_signal::Signal<void(Parameter*)> destroyed;
    slim_signal::Signal<void(ParameterPtr)> removed;

    slim_signal::Signal<void(const std::string&)> dictionary_entry_changed;

    static const uint8_t PACKET_TYPE_ID = 4;

public:
    virtual ~Parameter();

    void serialize_yaml(YAML::Node& n) const;
    void deserialize_yaml(const YAML::Node& n);

    virtual uint8_t getPacketType() const override;

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(const SerializationBuffer& data) override;

protected:
    virtual void doSerialize(YAML::Node& n) const = 0;
    virtual void doDeserialize(const YAML::Node& n) = 0;

public:
    std::string name() const;

    void setUUID(const UUID& uuid);
    UUID getUUID() const;

    virtual int ID() const = 0;
    virtual std::string TYPE() const = 0;


    template <typename T>
    bool is() const
    {
        return accepts(typeid(T));
    }

    virtual bool accepts(const std::type_info& type) const;

    Lock lock() const;

    template <typename T, typename std::enable_if<!std::is_enum<T>::value, int>::type = 0>
    T as() const
    {
        return as_impl<T>();
    }

    template <typename T, typename std::enable_if<std::is_enum<T>::value, int>::type = 0>
    T as() const
    {
        return static_cast<T>(as_impl<int>());
    }

    template <typename T>
    void set(const T& v)
    {
        if(!is<T>() && !is<void>()) {
            throwTypeError(typeid(T), type(),"set failed: ");
        }
        bool changed = setSilent(v);

        if(changed) {
            triggerChange();
        }
    }

    template <typename T>
    bool setSilent(const T& v);

    template <typename T>
    Parameter& operator = (const T& value)
    {
        set(value);
        return *this;
    }

    Parameter& operator = (const char* cstr)
    {
        return operator = (std::string(cstr));
    }

protected:
    template <typename T>
    T as_impl() const;

    Parameter& operator = (const Parameter& p);

public:
    virtual const std::type_info &type() const;
    std::string toString() const;

    const ParameterDescription& description() const;

    bool isEnabled() const;
    void setEnabled(bool enabled);

    bool isTemporary() const;
    void setTemporary(bool temporary);

    bool isHidden() const;
    void setHidden(bool hidden);

    bool isInteractive() const;
    void setInteractive(bool enabled);

    virtual bool hasState() const;

    void triggerChange();


    void setDictionaryEntry(const std::string& key, const ParameterPtr &param);
    ParameterPtr getDictionaryEntry(const std::string& key) const;

    template <typename T>
    void setDictionaryValue(const std::string& key, const T& value);
    template <typename T>
    T getDictionaryValue(const std::string& key)
    {
        return dict_.at(key)->as<T>();
    }
    template <typename T>
    T getDictionaryValue(const std::string& key, const T& def_value)
    {
        auto pos = dict_.find(key);
        if(pos == dict_.end()) {
            return def_value;
        } else {
            return pos->second->as<T>();
        }
    }

    virtual void get_unsafe(boost::any& out) const = 0;
    virtual bool set_unsafe(const boost::any& v) = 0;

public:
    static std::string type2string(const std::type_info& type);

protected:
    virtual std::string toStringImpl() const;
    void throwTypeError(const std::type_info& a, const std::type_info& b, const std::string& prefix) const;

protected:
    explicit Parameter(const std::string& name, const ParameterDescription& description);
    Parameter(const Parameter& other);

    void access_unsafe(const Parameter &p, boost::any& out) const;

private:
    void setName(const std::string& name);
    void setDescription(const ParameterDescription& desc);

protected:
    std::string name_;
    UUID uuid_;

    ParameterDescription description_;
    bool enabled_;
    bool temporary_;
    bool hidden_;
    bool interactive_;

    std::map<std::string, param::ParameterPtr> dict_;

    mutable std::recursive_mutex mutex_;
};

}
}

#endif // PARAMETER_H
