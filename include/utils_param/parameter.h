#ifndef PARAMETER_H
#define PARAMETER_H

/// COMPONENT
#include <utils_param/param_fwd.h>
#include <utils_param/parameter_description.h>

/// SYSTEM
#include <memory>
#include <boost/any.hpp>
#include <boost/signals2.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/recursive_mutex.hpp>

/// FORWARD DECLARATIONS
namespace YAML {
class Node;
}

namespace param {

class Parameter : boost::noncopyable
{
public:
    friend class ParameterFactory;

    typedef std::shared_ptr<Parameter> Ptr;

    typedef std::shared_ptr<boost::recursive_mutex::scoped_lock> Lock;

public:
    boost::signals2::signal<void(Parameter*)> parameter_changed;
    boost::signals2::signal<void(Parameter*)> scope_changed;
    boost::signals2::signal<void(Parameter*, bool)> interactive_changed;
    boost::signals2::signal<void(Parameter*, bool)> parameter_enabled;

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
        {
            Lock l = lock();
            set_unsafe(v);
        }
        triggerChange();
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

    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

    boost::any access_unsafe(const Parameter &p) const;

protected:
    std::string name_;
    std::string uuid_;

    ParameterDescription description_;
    bool enabled_;
    bool interactive_;

    mutable boost::recursive_mutex mutex_;
};

}

#endif // PARAMETER_H
