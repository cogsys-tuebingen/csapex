#ifndef VALUE_PARAMETER_H
#define VALUE_PARAMETER_H

/// COMPONENT
#include <utils_param/parameter.h>

/// SYSTEM
#include <boost/variant.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/contains.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/type_traits.hpp>

namespace param {


class ValueParameter : public Parameter
{
    friend class boost::serialization::access;
    friend class ParameterFactory;

public:
    typedef boost::any variant;

public:
    typedef boost::shared_ptr<ValueParameter> Ptr;

public:
    friend YAML::Emitter& operator << (YAML::Emitter& e, const ValueParameter& p) {
        p.write(e);
        return e;
    }
    friend YAML::Emitter& operator << (YAML::Emitter& e, const ValueParameter::Ptr& p) {
        p->write(e);
        return e;
    }

    friend void operator >> (const YAML::Node& node, param::ValueParameter& value) {
        value.read(node);
    }

    friend void operator >> (const YAML::Node& node, param::ValueParameter::Ptr& value) {
        if(!value) {
            value.reset(new ValueParameter("loading"));
        }
        value->read(node);
    }

public:
    ValueParameter();
    ValueParameter(const ValueParameter& rhs);
    explicit ValueParameter(const std::string& name);
    virtual ~ValueParameter();

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void setFrom(const Parameter& other);

    void write(YAML::Emitter& e) const;
    void read(const YAML::Node& n);

    template <typename T>
    T def() const { return read<T>(def_); }

    ValueParameter& operator = (const ValueParameter& param);

    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*file_version*/) {
        ar & value_;
    }

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

private:
    template <typename T>
    T read(const variant& var) const
    {
        try {
            return boost::any_cast<T> (var);

        } catch(const boost::bad_any_cast& e) {
            throw std::logic_error(std::string("typeof ValueParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    variant value_;
    variant def_;
};

}

#endif // VALUE_PARAMETER_H
