#ifndef PARAMETER_FACTORY_H
#define PARAMETER_FACTORY_H

/// COMPONENT
#include <utils_param/parameter.h>
#include <utils_param/range_parameter.h>
#include <utils_param/value_parameter.h>

namespace param
{

class ParameterFactory
{
public:
    template <typename T>
    static Parameter::Ptr declare(const std::string& name, T min, T max, T def, T step)
    {
        BOOST_STATIC_ASSERT((boost::mpl::contains<RangeParameterTypes, T>::value));

        RangeParameter::Ptr result(new RangeParameter(name));
        result->def_ = def;
        result->min_ = min;
        result->max_ = max;
        result->step_ = step;
        result->set<T>(def);

        return result;
    }

    static Parameter::Ptr declare(const std::string& name, bool def)
    {
        RangeParameter::Ptr result(new RangeParameter(name));
        result->def_ = def;
        result->set<bool>(def);

        return result;
    }

    template <typename T>
    static Parameter::Ptr declare(const std::string& name, const T& def)
    {
        ValueParameter::Ptr result(new ValueParameter(name));
        result->def_ = def;
        result->set<T>(def);

        return result;
    }
    static Parameter::Ptr declare(const std::string& name, const char* def)
    {
        return declare(name, std::string(def));
    }

    static Parameter::Ptr makeEmpty(const std::string& type);
};

}


#endif // PARAMETER_FACTORY_H
