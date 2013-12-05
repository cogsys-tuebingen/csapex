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

class Parameter
{
public:
    typedef boost::shared_ptr<Parameter> Ptr;

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
        (*parameter_changed)(this);
    }

    template <typename T>
    Parameter& operator = (const T& value)
    {
        boost::any v = value;
        set_unsafe(v);
        (*parameter_changed)(this);
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

protected:
    virtual std::string toStringImpl() const;
    std::string type2string(const std::type_info& type) const;
    void throwTypeError(const std::type_info& a, const std::type_info& b, const std::string& prefix) const;

protected:
    explicit Parameter(const std::string& name);

    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

public:
    boost::shared_ptr<boost::signals2::signal<void(Parameter*)> > parameter_changed;

protected:
    std::string name_;
};

}

#endif // PARAMETER_H
