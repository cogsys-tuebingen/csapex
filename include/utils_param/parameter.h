#ifndef PARAMETER_H
#define PARAMETER_H

/// SYSTEM
#include <stdexcept>
#include <utils_yaml/yamlplus.h>
#include <boost/shared_ptr.hpp>
#include <boost/any.hpp>
#include <boost/signals2.hpp>
#include <cxxabi.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

namespace param {

class Parameter : boost::noncopyable
{
public:
    friend class ParameterFactory;

    typedef boost::shared_ptr<Parameter> Ptr;

    typedef boost::shared_ptr<boost::lock_guard<boost::mutex> > Lock;

    struct access {
        boost::signals2::signal<void(Parameter*)>& parameter_changed(Parameter& p) {
            return p.parameter_changed;
        }
        boost::signals2::signal<void(Parameter*)>& scope_changed(Parameter& p) {
            return p.scope_changed;
        }
        boost::signals2::signal<void(Parameter*,bool)>& interactive_changed(Parameter& p) {
            return p.interactive_changed;
        }
        boost::signals2::signal<void(Parameter*,bool)>& parameter_enabled(Parameter& p) {
            return p.parameter_enabled;
        }
    };

public:
    virtual ~Parameter();

    void write(YAML::Emitter& e) const;
    void read(const YAML::Node& n);

    void setValueFrom(const Parameter& other);
    void clone(const Parameter& other);

protected:
    virtual void doWrite(YAML::Emitter& e) const = 0;
    virtual void doRead(const YAML::Node& n) = 0;
    virtual void doSetValueFrom(const Parameter& other) = 0;
    virtual void doClone(const Parameter& other) = 0;

public:
    std::string name() const;

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

    bool isEnabled() const;
    void setEnabled(bool enabled);

    bool isInteractive() const;
    void setInteractive(bool enabled);

    void triggerChange();

public:
    static std::string type2string(const std::type_info& type);

protected:
    virtual std::string toStringImpl() const;
    void throwTypeError(const std::type_info& a, const std::type_info& b, const std::string& prefix) const;

protected:
    explicit Parameter(const std::string& name);

    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

    boost::any access_unsafe(const Parameter &p) const;

protected:
    boost::signals2::signal<void(Parameter*)> parameter_changed;
    boost::signals2::signal<void(Parameter*)> scope_changed;
    boost::signals2::signal<void(Parameter*, bool)> interactive_changed;
    boost::signals2::signal<void(Parameter*, bool)> parameter_enabled;

protected:
    std::string name_;
    bool enabled_;
    bool interactive_;

    mutable boost::mutex mutex_;
};

}

#endif // PARAMETER_H
