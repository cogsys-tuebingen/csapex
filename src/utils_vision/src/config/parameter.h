#ifndef PARAMETER_H
#define PARAMETER_H

/// SYSTEM
#include <boost/variant.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/contains.hpp>
#include <stdexcept>
#include <boost/serialization/variant.hpp>
#include <boost/type_traits.hpp>
#include <yaml-cpp/yaml.h>

namespace vision {

typedef boost::mpl::vector<
std::string,
double,
int,
bool
> ParameterTypes;


class Parameter
{
    friend class boost::serialization::access;

    typedef boost::make_variant_over<ParameterTypes>::type variant;

public:
    friend YAML::Emitter& operator << (YAML::Emitter& e, const Parameter& p) {
        p.write(e);
        return e;
    }

    friend void operator >> (const YAML::Node& node, vision::Parameter& value) {
        value.read(node);
    }

public:
    Parameter();
    explicit Parameter(const std::string& name);

    void setFrom(const Parameter& other);

    void write(YAML::Emitter& e) const;
    void read(const YAML::Node& n);

    std::string name() const;

    template <typename T>
    operator T() const
    {
        BOOST_STATIC_ASSERT((boost::mpl::contains<ParameterTypes, T>::value));
        return as<T> ();
    }

    template <typename T>
    bool is() const
    {
        return value_.type() == typeid(T);
    }

    template <typename T>
    T as() const { return read<T>(value_); }

    template <typename T>
    void set(T value)
    {
        BOOST_STATIC_ASSERT((boost::mpl::contains<ParameterTypes, T>::value));
        value_ = value;
    }

    template <typename T>
    T min() const { return read<T>(min_); }

    template <typename T>
    T max() const { return read<T>(max_); }

    template <typename T>
    T def() const { return read<T>(def_); }

    template <typename T>
    T step() const { return read<T>(step_); }

    template <typename T>
    Parameter& operator = (const T& value)
    {
        value_ = value;
        return *this;
    }

    Parameter& operator = (const char* value)
    {
        return operator = (std::string(value));
    }

    template <typename T>
    static Parameter declare(const std::string& name, T min, T max, T def, T step)
    {
        BOOST_STATIC_ASSERT((boost::mpl::contains<ParameterTypes, T>::value));

        Parameter result(name);
        result.def_ = def;
        result.min_ = min;
        result.max_ = max;
        result.value_ = def;
        result.step_ = step;
        return result;
    }

    static Parameter declare(const std::string& name, bool def)
    {
        Parameter result(name);
        result.def_ = def;
        result.value_ = def;
        return result;
    }

    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*file_version*/) {
        ar & value_;
    }

private:
    template <typename T>
    T read(const variant& var) const
    {
        BOOST_STATIC_ASSERT((boost::mpl::contains<ParameterTypes, T>::value));
        try {
            return boost::get<T>(var);

        } catch(const boost::bad_get& e) {
            throw std::logic_error(std::string("typeof parameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    std::string name_;

    variant value_;
    variant min_;
    variant max_;
    variant def_;
    variant step_;
};

}

#endif // PARAMETER_H
