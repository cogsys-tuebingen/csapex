#ifndef RANGE_PARAMETER_H
#define RANGE_PARAMETER_H

/// COMPONENT
#include <utils_param/parameter.h>

/// SYSTEM
#include <boost/mpl/vector.hpp>
#include <boost/mpl/contains.hpp>

namespace param {

typedef boost::mpl::vector<
double,
int
> RangeParameterTypes;


class RangeParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef boost::shared_ptr<RangeParameter> Ptr;

public:
    RangeParameter();
    explicit RangeParameter(const std::string& name, const ParameterDescription &description);
    virtual ~RangeParameter();

    virtual int ID() const { return 0x005; }
    virtual std::string TYPE() const { return "range"; }

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);

    void doSerialize(YAML::Node& e) const;
    void doDeserialize(const YAML::Node& n);

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
        T _min = read<T>(min_);
        if(min != _min) {
            min_ = min;
            T _max = read<T>(max_);
            if(_max < min) {
                max_ = min;
            }
            scope_changed(this);
        }
    }

    template <typename T>
    void setMax(T max) {
        T _max = read<T>(max_);
        if(_max != max) {
            max_ = max;
            T _min = read<T>(min_);
            if(max < _min) {
                min_ = max;
            }
            scope_changed(this);
        }
    }

//    template <typename T>
//    void setStep(T step) {
//        T _step = read<T>(step_);
//        if (_step != step) {
//            // test, if difference between max and min is bigger than step
//            T _max = read<T>(max_);
//            T _min = read<T>(min_);
//            if(((_min + step) < _max) && ((_min - step) < _max)) {
//                step_ = step;
//            } else {
//                step_ = _max - _min;
//            }
//        }
//    }

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

private:
    template <typename T>
    T read(const boost::any& var) const
    {
        BOOST_STATIC_ASSERT((boost::mpl::contains<RangeParameterTypes, T>::value));
        try {
            return boost::any_cast<T> (var);

        } catch(const boost::bad_any_cast& e) {
            throw std::logic_error(std::string("typeof RangeParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    boost::any value_;
    boost::any min_;
    boost::any max_;
    boost::any def_;
    boost::any step_;
};

}

#endif // RANGE_PARAMETER_H
