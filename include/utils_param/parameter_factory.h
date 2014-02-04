#ifndef PARAMETER_FACTORY_H
#define PARAMETER_FACTORY_H

/// COMPONENT
#include <utils_param/parameter.h>
#include <utils_param/range_parameter.h>
#include <utils_param/value_parameter.h>
#include <utils_param/set_parameter.h>
#include <utils_param/interval_parameter.h>

namespace param
{

class ParameterFactory
{
public:
    static Parameter::Ptr declareBool(const std::string& name, bool def);

    template <typename T>
    static Parameter::Ptr declareRange(const std::string& name, T min, T max, T def, T step)
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

    template <typename T>
    static Parameter::Ptr declareInterval(const std::string& name, T min, T max, T def_min, T def_max, T step)
    {
        BOOST_STATIC_ASSERT((boost::mpl::contains<IntervalParameterTypes, T>::value));

        IntervalParameter::Ptr result(new IntervalParameter(name));
        result->def_ = std::make_pair(def_min, def_max);
        result->min_ = min;
        result->max_ = max;
        result->step_ = step;
        result->values_ = std::make_pair(def_min, def_max);

        result->set<std::pair<T,T> >(std::make_pair(def_min, def_max));

        return result;
    }

    static Parameter::Ptr declarePath(const std::string& name, const std::string& def);

    static Parameter::Ptr declareTrigger(const std::string& name);

    static Parameter::Ptr declareColorParameter(const std::string& name, int r, int g, int b);

    static Parameter::Ptr makeEmpty(const std::string& type);

    template <typename T>
    static Parameter::Ptr declareParameterSet(const std::string& name, const std::vector< std::pair<std::string, T> >& set)
    {
        SetParameter::Ptr result(new SetParameter(name));
        result->setSet(set);
        result->def_ = set.begin()->second;
        result->set<T>(set.begin()->second);

        return result;
    }

    static Parameter::Ptr declareParameterBitSet(const std::string& name, const std::vector< std::pair<std::string, int> >& set);

    template <typename T>
    static Parameter::Ptr declareValue(const std::string& name, const T& def)
    {
        ValueParameter::Ptr result(new ValueParameter(name));
        result->def_ = def;
        result->set<T>(def);

        return result;
    }

    /** LEGACY **/
    template <typename T>
    static Parameter::Ptr declare(const std::string& name, T min, T max, T def, T step)
    {
        return declareRange(name, min, max, def, step);
    }

    static Parameter::Ptr declare(const std::string& name, bool def);

    template <typename T>
    static Parameter::Ptr declare(const std::string& name, const T& def)
    {
        return declareValue(name, def);
    }

    static Parameter::Ptr declare(const std::string& name, const char* def);

    /** / LEGACY **/
};

}


#endif // PARAMETER_FACTORY_H
