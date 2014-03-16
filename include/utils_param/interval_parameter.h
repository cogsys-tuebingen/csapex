#ifndef INTERVAL_PARAMETER_H
#define INTERVAL_PARAMETER_H

/// COMPONENT
#include <utils_param/parameter.h>

/// SYSTEM
#include <boost/variant.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/contains.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/type_traits.hpp>

namespace param {

typedef boost::mpl::vector<
double,
int
> IntervalParameterTypes;


class IntervalParameter : public Parameter
{
    friend class boost::serialization::access;
    friend class ParameterFactory;

public:
    typedef boost::any variant;

public:
    typedef boost::shared_ptr<IntervalParameter> Ptr;

public:
    friend YAML::Emitter& operator << (YAML::Emitter& e, const IntervalParameter& p) {
        p.write(e);
        return e;
    }
    friend YAML::Emitter& operator << (YAML::Emitter& e, const IntervalParameter::Ptr& p) {
        p->write(e);
        return e;
    }

    friend void operator >> (const YAML::Node& node, param::IntervalParameter& value) {
        value.read(node);
    }

    friend void operator >> (const YAML::Node& node, param::IntervalParameter::Ptr& value) {
        if(!value) {
            value.reset(new IntervalParameter("loading"));
        }
        value->read(node);
    }

public:
    IntervalParameter();
    explicit IntervalParameter(const std::string& name);
    virtual ~IntervalParameter();

    virtual int ID() const { return 0x003; }
    virtual std::string TYPE() const { return "Interval"; }

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void setFrom(const Parameter& other);

    void write(YAML::Emitter& e) const;
    void read(const YAML::Node& n);

    template <typename T>
    void setLower(T v) {
        values_.first = v;
        parameter_changed(this);
    }
    template <typename T>
    void setUpper(T v) {
        values_.second = v;
        parameter_changed(this);
    }

    template <typename T>
    T lower() const { return read<T>(values_.first); }

    template <typename T>
    T upper() const { return read<T>(values_.second); }

    template <typename T>
    T min() const { return read<T>(min_); }

    template <typename T>
    T max() const { return read<T>(max_); }

    template <typename T>
    T def() const { return read<T>(def_); }

    template <typename T>
    T step() const { return read<T>(step_); }

    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*file_version*/) {
        ar & values_;
    }

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

private:
    template <typename T>
    T read(const variant& var) const
    {
        BOOST_STATIC_ASSERT((boost::mpl::contains<IntervalParameterTypes, T>::value));
        try {
            return boost::any_cast<T> (var);

        } catch(const boost::bad_any_cast& e) {
            throw std::logic_error(std::string("typeof IntervalParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    std::pair<variant, variant> values_;
    variant min_;
    variant max_;
    std::pair<variant, variant> def_;
    variant step_;
};

}

#endif // INTERVAL_PARAMETER_H
