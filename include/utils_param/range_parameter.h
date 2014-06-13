#ifndef RANGE_PARAMETER_H
#define RANGE_PARAMETER_H

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
> RangeParameterTypes;


class RangeParameter : public Parameter
{
    friend class boost::serialization::access;
    friend class ParameterFactory;

public:
    typedef boost::any variant;

public:
    typedef boost::shared_ptr<RangeParameter> Ptr;

public:
    friend YAML::Emitter& operator << (YAML::Emitter& e, const RangeParameter& p) {
        p.doWrite(e);
        return e;
    }
    friend YAML::Emitter& operator << (YAML::Emitter& e, const RangeParameter::Ptr& p) {
        p->doWrite(e);
        return e;
    }

    friend void operator >> (const YAML::Node& node, param::RangeParameter& value) {
        value.doRead(node);
    }

    friend void operator >> (const YAML::Node& node, param::RangeParameter::Ptr& value) {
        if(!value) {
            value.reset(new RangeParameter("loading", ParameterDescription("")));
        }
        value->doRead(node);
    }

public:
    RangeParameter();
    explicit RangeParameter(const std::string& name, const ParameterDescription &description);
    virtual ~RangeParameter();

    virtual int ID() const { return 0x005; }
    virtual std::string TYPE() const { return "Range"; }

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);

    void doWrite(YAML::Emitter& e) const;
    void doRead(const YAML::Node& n);

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
        ar & value_;
    }

    template <typename T>
    void setInterval(T min, T max) {
        if(min != read<T>(min_) || max != read<T>(max_)) {
            min_ = min;
            max_ = max;
            scope_changed(this);
        }
    }

    template <typename T>
    void setMin(T min) {
        if(min != read<T>(min_)) {
            min_ = min; scope_changed(this);
        }
    }

    template <typename T>
    void setMax(T max) {
        if(read<T>(max_) != max) {
            max_ = max;
            scope_changed(this);
        }
    }
protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

private:
    template <typename T>
    T read(const variant& var) const
    {
        BOOST_STATIC_ASSERT((boost::mpl::contains<RangeParameterTypes, T>::value));
        try {
            return boost::any_cast<T> (var);

        } catch(const boost::bad_any_cast& e) {
            throw std::logic_error(std::string("typeof RangeParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    variant value_;
    variant min_;
    variant max_;
    variant def_;
    variant step_;
};

}

#endif // RANGE_PARAMETER_H
