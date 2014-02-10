#ifndef PARAMETER_H
#define PARAMETER_H

/// SYSTEM
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>
#include <boost/any.hpp>
#include <boost/signals2.hpp>
#include <cxxabi.h>

namespace param {

class Parameter : boost::noncopyable
{
public:
    typedef boost::shared_ptr<Parameter> Ptr;

    struct access {
        boost::signals2::signal<void(Parameter*)>& parameter_changed(Parameter& p) {
            return p.parameter_changed;
        }
        boost::signals2::signal<void(Parameter*,bool)>& parameter_enabled(Parameter& p) {
            return p.parameter_enabled;
        }
    };

public:
    virtual ~Parameter();

    virtual void write(YAML::Emitter& e) const;
    virtual void read(const YAML::Node& n);

    std::string name() const;


    template <typename T>
    bool is() const
    {
        return type() == typeid(T);
    }

    template <typename T>
    T as() const
    {
        if(!is<T>() || is<void>()) {
            throwTypeError(type(), typeid(T), "get failed: ");
        }
        const boost::any& v = get_unsafe();
        return boost::any_cast<T> (v);
    }

    template <typename T>
    void set(const T& v)
    {
        if(!is<T>() && !is<void>()) {
            throwTypeError(type(), typeid(T), "set failed: ");
        }

        set_unsafe(v);
        parameter_changed(this);
    }

    template <typename T>
    Parameter& operator = (const T& value)
    {
        boost::any v = value;
        set_unsafe(v);
        parameter_changed(this);
        return *this;
    }

    Parameter& operator = (const char* cstr)
    {
        return operator = (std::string(cstr));
    }

    static Parameter::Ptr empty();

    virtual const std::type_info &type() const;
    std::string toString() const;

    virtual void setFrom(const Parameter& other);

    bool isEnabled() const;
    void setEnabled(bool enabled);


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
    boost::signals2::signal<void(Parameter*, bool)> parameter_enabled;

protected:
    std::string name_;
    bool enabled_;
};

}

#endif // PARAMETER_H
