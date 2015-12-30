#ifndef PARAMETER_H
#define PARAMETER_H

/// COMPONENT
#include <csapex/param/param_fwd.h>
#include <csapex/param/parameter_description.h>

/// SYSTEM
#include <memory>
#include <boost/any.hpp>
#include <csapex/utility/slim_signal.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/recursive_mutex.hpp>

/// FORWARD DECLARATIONS
namespace YAML {
class Node;
}

namespace csapex {
namespace param {

class Parameter : boost::noncopyable
{
public:
    friend class ParameterFactory;
    friend class ParameterBuilder;

    typedef std::shared_ptr<Parameter> Ptr;

    typedef std::shared_ptr<boost::recursive_mutex::scoped_lock> Lock;

public:
    csapex::slim_signal::Signal<void(Parameter*)> parameter_changed;
    csapex::slim_signal::Signal<void(Parameter*)> scope_changed;
    csapex::slim_signal::Signal<void(Parameter*, bool)> interactive_changed;
    csapex::slim_signal::Signal<void(Parameter*, bool)> parameter_enabled;
    csapex::slim_signal::Signal<void(Parameter*)> destroyed;

public:
    virtual ~Parameter();

    void serialize(YAML::Node& n) const;
    void deserialize(const YAML::Node& n);

    void setValueFrom(const Parameter& other);
    void clone(const Parameter& other);

protected:
    virtual void doSerialize(YAML::Node& n) const = 0;
    virtual void doDeserialize(const YAML::Node& n) = 0;

    virtual void doSetValueFrom(const Parameter& other) = 0;
    virtual void doClone(const Parameter& other) = 0;

public:
    std::string name() const;

    void setUUID(const std::string& uuid);
    std::string UUID() const;

    virtual int ID() const = 0;
    virtual std::string TYPE() const = 0;


    template <typename T>
    bool is() const
    {
        return type() == typeid(T);
    }

    Lock lock() const;

    template <typename T>
    T as() const
    {
        if(!is<T>() || is<void>()) {
            throwTypeError(typeid(T), type(), "get failed: ");
        }

        {
            Lock l = lock();
            const boost::any& v = get_unsafe();
            return boost::any_cast<T> (v);
        }
    }

    template <typename T>
    void set(const T& v)
    {
        if(!is<T>() && !is<void>()) {
            throwTypeError(typeid(T), type(),"set failed: ");
        }
        bool changed = false;
        {
            Lock l = lock();
            changed = set_unsafe(v);
        }

        if(changed) {
            triggerChange();
        }
    }

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

    virtual const std::type_info &type() const;
    std::string toString() const;

    const ParameterDescription& description() const;

    bool isEnabled() const;
    void setEnabled(bool enabled);

    bool isInteractive() const;
    void setInteractive(bool enabled);

    virtual bool hasState() const;

    void triggerChange();

public:
    static std::string type2string(const std::type_info& type);

protected:
    virtual std::string toStringImpl() const;
    void throwTypeError(const std::type_info& a, const std::type_info& b, const std::string& prefix) const;

protected:
    explicit Parameter(const std::string& name, const ParameterDescription& description);

    virtual boost::any get_unsafe() const = 0;
    virtual bool set_unsafe(const boost::any& v) = 0;

    boost::any access_unsafe(const Parameter &p) const;

private:
    void setName(const std::string& name);
    void setDescription(const ParameterDescription& desc);

protected:
    std::string name_;
    std::string uuid_;

    ParameterDescription description_;
    bool enabled_;
    bool interactive_;

    mutable boost::recursive_mutex mutex_;
};

}
}

#endif // PARAMETER_H
