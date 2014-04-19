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
    static Parameter::Ptr makeEmpty(const std::string& type);
    static Parameter::Ptr clone(const Parameter::Ptr& param);



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

    static Parameter::Ptr declareText(const std::string& name, const std::string& def);

    static Parameter::Ptr declarePath(const std::string& name, bool is_file, const std::string& def, const std::string& filter = "", bool input = false, bool output = false);

    static Parameter::Ptr declareFileInputPath(const std::string& name, const std::string& def, const std::string& filter = "");
    static Parameter::Ptr declareFileOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");
    static Parameter::Ptr declareFileInputOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");

    static Parameter::Ptr declareDirectoryInputPath(const std::string& name, const std::string& def, const std::string& filter = "");
    static Parameter::Ptr declareDirectoryOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");
    static Parameter::Ptr declareDirectoryInputOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");

    static Parameter::Ptr declareTrigger(const std::string& name);

    static Parameter::Ptr declareColorParameter(const std::string& name, int r, int g, int b);

    template <typename T>
    static Parameter::Ptr declareParameterSet(const std::string& name, const std::map<std::string, T> & set)
    {
        SetParameter::Ptr result(new SetParameter(name));
        result->setSet(set);
        if(!set.empty()) {
            result->def_ = set.begin()->second;
            result->set<T>(set.begin()->second);
        }

        return result;
    }

    static Parameter::Ptr declareParameterStringSet(const std::string& name, const std::vector<std::string> & set)
    {
        SetParameter::Ptr result(new SetParameter(name));
        result->setSet(set);

        if(!set.empty()) {
            result->def_ = set.begin();
            result->set<std::string>(*set.begin());
        }

        return result;
    }

    static Parameter::Ptr declareParameterBitSet(const std::string& name, const std::map<std::string, int> &set, int def = 0);
    static Parameter::Ptr declareParameterBitSet(const std::string& name, const std::map<std::string, std::pair<int, bool> > &set);

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
    static Parameter::Ptr declareParameterSet(const std::string& name, const std::vector< std::pair<std::string, T> >& set)
    {
        SetParameter::Ptr result(new SetParameter(name));
        result->setSet(set);
        result->def_ = set.begin()->second;
        result->set<T>(set.begin()->second);

        return result;
    }

    template <typename T>
    static Parameter::Ptr declare(const std::string& name, T min, T max, T def, T step)
    {
        return declareRange(name, min, max, def, step);
    }

    static Parameter::Ptr declare(const std::string& name, bool def) __attribute__ ((deprecated));

    template <typename T>
    static Parameter::Ptr declare(const std::string& name, const T& def)
    {
        return declareValue(name, def);
    }

    static Parameter::Ptr declare(const std::string& name, const char* def)  __attribute__ ((deprecated));

    /** / LEGACY **/
};

}


#endif // PARAMETER_FACTORY_H
