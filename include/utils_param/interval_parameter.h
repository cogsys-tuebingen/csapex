#ifndef INTERVAL_PARAMETER_H
#define INTERVAL_PARAMETER_H

/// COMPONENT
#include <utils_param/parameter.h>

/// SYSTEM
#include <boost/mpl/vector.hpp>
#include <boost/mpl/contains.hpp>

namespace param {

typedef boost::mpl::vector<
double,
int
> IntervalParameterTypes;


class IntervalParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<IntervalParameter> Ptr;

public:
    IntervalParameter();
    explicit IntervalParameter(const std::string& name, const ParameterDescription &description);
    virtual ~IntervalParameter();

    virtual int ID() const { return 0x003; }
    virtual std::string TYPE() const { return "interval"; }

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);

    void doSerialize(YAML::Node& e) const;
    void doDeserialize(const YAML::Node& n);

    template <typename T>
    void setLower(T v) {
        Lock l = lock();
        values_.first = v;
        parameter_changed(this);
    }
    template <typename T>
    void setUpper(T v) {
        Lock l = lock();
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


    template <typename T>
    void setInterval(T min, T max) {
        Lock l = lock();
        if(min != read<T>(min_) || max != read<T>(max_)) {
            min_ = min;
            max_ = max;
            scope_changed(this);
        }
    }

    template <typename T>
    void setMin(T min) {
        if(min != read<T>(min_)) {
            Lock l = lock();
            min_ = min; scope_changed(this);
        }
    }

    template <typename T>
    void setMax(T max) {
        if(read<T>(max_) != max) {
            Lock l = lock();
            max_ = max;
            scope_changed(this);
        }
    }

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

private:
    template <typename T>
    T read(const boost::any& var) const
    {
        BOOST_STATIC_ASSERT((boost::mpl::contains<IntervalParameterTypes, T>::value));
        try {
            Lock l = lock();
            return boost::any_cast<T> (var);

        } catch(const boost::bad_any_cast& e) {
            throw std::logic_error(std::string("typeof IntervalParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    std::pair<boost::any, boost::any> values_;
    boost::any min_;
    boost::any max_;
    std::pair<boost::any, boost::any> def_;
    boost::any step_;
};

}

#endif // INTERVAL_PARAMETER_H
