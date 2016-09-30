/// HEADER
#include <csapex/param/parameter_factory.h>

/// COMPONENT
#include <csapex/param/angle_parameter.h>
#include <csapex/param/color_parameter.h>
#include <csapex/param/path_parameter.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/param/bitset_parameter.h>
#include <csapex/param/range_parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/param/string_list_parameter.h>
#include <csapex/param/output_progress_parameter.h>
#include <csapex/param/output_text_parameter.h>
#include <csapex/param/set_parameter.h>

/// SYSTEM
#include <iostream>

using namespace csapex;
using namespace param;

ParameterBuilder ParameterFactory::makeEmpty(const std::string &type)
{
    std::string t = type;
    std::transform(t.begin(), t.end(), t.begin(), tolower);
    if(t == "range") {
        return std::shared_ptr<Parameter>(new RangeParameter);
    } else if(t == "interval") {
        return std::shared_ptr<Parameter>(new IntervalParameter);
    } else if(t == "value") {
        return std::shared_ptr<Parameter>(new ValueParameter);
    } else if(t == "set") {
        return std::shared_ptr<Parameter>(new SetParameter);
    } else if(t == "bitset") {
        return std::shared_ptr<Parameter>(new BitSetParameter);
    } else if(t == "path") {
        return std::shared_ptr<Parameter>(new PathParameter);
    } else if(t == "trigger") {
        return std::shared_ptr<Parameter>(new TriggerParameter);
    } else if(t == "string_list") {
        return std::shared_ptr<Parameter>(new StringListParameter);
    } else if(t == "color") {
        return std::shared_ptr<Parameter>(new ColorParameter);
    } else if(t == "angle") {
        return std::shared_ptr<Parameter>(new AngleParameter);
    } else if(t == "progress") {
        return std::shared_ptr<Parameter>(new OutputProgressParameter);
    } else if(t == "outtext") {
        return std::shared_ptr<Parameter>(new OutputTextParameter);
    } else {
        throw std::runtime_error(std::string("illegal parameter type: ") + t);
    }
}

ParameterBuilder ParameterFactory::clone(const Parameter* param)
{
    std::string type = param->TYPE();
    ParameterBuilder r = makeEmpty(type);
    ParameterPtr p = r.build();
    p->clone(*param);
    return r;
}
ParameterBuilder ParameterFactory::clone(const Parameter& param)
{
    return clone(&param);
}
ParameterBuilder ParameterFactory::clone(const std::shared_ptr<Parameter>& param)
{
    return clone(param.get());
}


ParameterBuilder ParameterFactory::declareParameterBitSet(const std::string &name, const ParameterDescription& description, const std::map<std::string, int> &set, int def)
{
    std::shared_ptr<BitSetParameter> result(new BitSetParameter(name, description));
    result->setBitSet(set);
    result->def_ = def;
    result->set<int>(def);

    return ParameterBuilder(std::move(result));
}

ParameterBuilder ParameterFactory::declareParameterBitSet(const std::string &name, const std::map<std::string, int> &set, int def)
{
    return declareParameterBitSet(name, ParameterDescription(), set, def);
}

ParameterBuilder ParameterFactory::declareParameterBitSet(const std::string &name, const ParameterDescription& description, const std::map<std::string, std::pair<int, bool> > &set)
{
    std::map<std::string, int> raw_set;
    int def = 0;

    for(std::map<std::string, std::pair<int, bool> >::const_iterator it = set.begin();
        it != set.end();
        ++it)
    {
        raw_set[it->first] = it->second.first;
        if(it->second.second) {
            def += it->second.first;
        }
    }
    return declareParameterBitSet(name, description, raw_set, def);
}

ParameterBuilder ParameterFactory::declareParameterBitSet(const std::string &name, const std::map<std::string, std::pair<int, bool> > &set)
{
    return ParameterFactory::declareParameterBitSet(name, ParameterDescription(), set);
}

ParameterBuilder ParameterFactory::declareBool(const std::string& name, const ParameterDescription& description, bool def)
{
    std::shared_ptr<ValueParameter> result(new ValueParameter(name, description));
    result->def_ = def;
    result->set<bool>(def);

    return ParameterBuilder(std::move(result));
}

ParameterBuilder ParameterFactory::declareBool(const std::string &name, bool def)
{
    return declareBool(name, ParameterDescription(), def);
}

ParameterBuilder ParameterFactory::declareAngle(const std::string& name, const ParameterDescription& description, double angle)
{
    std::shared_ptr<AngleParameter> result(new AngleParameter(name, description, angle));
    return ParameterBuilder(std::move(result));
}

ParameterBuilder ParameterFactory::declareAngle(const std::string& name, double angle)
{
    return declareAngle(name, ParameterDescription(), angle);
}

ParameterBuilder ParameterFactory::declareColorParameter(const std::string& name, const ParameterDescription& description, int r, int g, int b)
{
    std::shared_ptr<ColorParameter> result(new ColorParameter(name, description, r, g, b));
    return ParameterBuilder(std::move(result));
}

ParameterBuilder ParameterFactory::declareColorParameter(const std::string& name, int r, int g, int b)
{
    return declareColorParameter(name, ParameterDescription(), r,g,b);
}

ParameterBuilder ParameterFactory::declareTrigger(const std::string& name, const ParameterDescription& description)
{
    std::shared_ptr<TriggerParameter> result(new TriggerParameter(name, description));
    return ParameterBuilder(std::move(result));
}

ParameterBuilder ParameterFactory::declareTrigger(const std::string &name)
{
    return declareTrigger(name, ParameterDescription());
}

ParameterBuilder ParameterFactory::declarePath(const std::string& name, const ParameterDescription& description,
                                             bool is_file, const std::string& def, const std::string& filter, bool input, bool output)
{
    std::shared_ptr<PathParameter> result(new PathParameter(name, description, filter, is_file, input, output));
    result->set(def);

    return ParameterBuilder(std::move(result));
}

ParameterBuilder ParameterFactory::declareFileInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, true, false);
}
ParameterBuilder ParameterFactory::declareFileInputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileInputPath(name, ParameterDescription(), def, filter);
}

ParameterBuilder ParameterFactory::declareFileOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, false, true);
}

ParameterBuilder ParameterFactory::declareFileOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileOutputPath(name, ParameterDescription(), def, filter);
}

ParameterBuilder ParameterFactory::declareFileInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, true, true);
}
ParameterBuilder ParameterFactory::declareFileInputOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileInputOutputPath(name, ParameterDescription(), def, filter);
}


ParameterBuilder ParameterFactory::declareDirectoryInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, true, false);
}

ParameterBuilder ParameterFactory::declareDirectoryInputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryInputPath(name, ParameterDescription(), def, filter);
}

ParameterBuilder ParameterFactory::declareDirectoryOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, false, true);
}
ParameterBuilder ParameterFactory::declareDirectoryOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryOutputPath(name, ParameterDescription(), def, filter);
}

ParameterBuilder ParameterFactory::declareDirectoryInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, true, true);
}
ParameterBuilder ParameterFactory::declareDirectoryInputOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryInputOutputPath(name, ParameterDescription(), def, filter);
}


ParameterBuilder ParameterFactory::declareText(const std::string& name, const ParameterDescription& description, const std::string& def)
{
    std::shared_ptr<ValueParameter> result(new ValueParameter(name, description));
    result->set(def);

    return ParameterBuilder(std::move(result));
}

ParameterBuilder ParameterFactory::declareText(const std::string &name, const std::string &def)
{
    return declareText(name, ParameterDescription(""), def);
}

ParameterBuilder ParameterFactory::declareParameterStringSet(const std::string& name, const ParameterDescription& description,
                                                           const std::vector<std::string> & set,
                                                           const std::string &def)
{
    std::shared_ptr<SetParameter> result(new SetParameter(name, description));
    result->setSet(set);

    std::string v = def;
    if(!set.empty()) {
        if(v.empty()) {
            v = set[0];
        }
        result->def_ = v;
        result->set<std::string>(v);
    }

    return ParameterBuilder(std::move(result));
}

ParameterBuilder ParameterFactory::declareParameterStringSet(const std::string& name, const std::vector<std::string> & set, const std::string& def)
{
    return declareParameterStringSet(name, ParameterDescription(), set, def);
}



ParameterBuilder ParameterFactory::declareOutputProgress(const std::string &name, const ParameterDescription &description)
{
    std::shared_ptr<OutputProgressParameter> result(new OutputProgressParameter(name, description));
    return ParameterBuilder(std::move(result));
}

ParameterBuilder ParameterFactory::declareOutputText(const std::string &name, const ParameterDescription &description)
{
    std::shared_ptr<OutputTextParameter> result(new OutputTextParameter(name, description));
    return ParameterBuilder(std::move(result));
}


namespace csapex
{
namespace param
{

template <typename T>
ParameterBuilder ParameterFactory::declareRange(const std::string& name,
                                   const ParameterDescription& description,
                                   T min, T max, T def, T step)
{
    BOOST_STATIC_ASSERT((boost::mpl::contains<RangeParameterTypes, T>::value));

    step = param::range::limitStep<T>(min, max, step);

    std::shared_ptr<RangeParameter> result(new RangeParameter(name, description));
    result->def_value_ = def;
    result->def_min_ = min;
    result->def_max_ = max;
    result->min_ = result->def_min_;
    result->max_ = result->def_max_;
    result->step_ = step;
    result->set<T>(def);

    return ParameterBuilder(std::move(result));
}

template <typename T>
ParameterBuilder ParameterFactory::declareInterval(const std::string& name,
                                                 const ParameterDescription& description,
                                                 T min, T max, T def_min, T def_max, T step)
{
    BOOST_STATIC_ASSERT((boost::mpl::contains<IntervalParameterTypes, T>::value));

    std::shared_ptr<IntervalParameter> result(new IntervalParameter(name, description));
    result->def_ = std::make_pair(def_min, def_max);
    result->min_ = min;
    result->max_ = max;
    result->step_ = step;
    result->values_ = std::make_pair(def_min, def_max);

    result->set<std::pair<T,T> >(std::make_pair(def_min, def_max));

    return ParameterBuilder(std::move(result));
}


template <typename T>
ParameterBuilder ParameterFactory::declareParameterSet(const std::string& name, const ParameterDescription& description,
                                          const std::map<std::string, T> & set_values,
                                          const T& def)
{
    std::unique_ptr<SetParameter> result(new SetParameter(name, description));
    result->setSet(set_values);
    if(!set_values.empty()) {
        result->def_ = def;
        result->set<T>(def);
    }

    return ParameterBuilder(std::move(result));
}
template <typename T>
ParameterBuilder ParameterFactory::declareValue(const std::string& name, const ParameterDescription& description, const T& def)
{
    std::unique_ptr<ValueParameter> result(new ValueParameter(name, description));
    result->def_ = def;
    result->set<T>(def);

    return ParameterBuilder(std::move(result));
}

/// EXPLICIT INSTANTIATON

namespace {
template<typename T> struct argument_type;
template<typename T, typename U> struct argument_type<T(U)> { typedef U type; };
}

#define INSTANTIATE(T) \
    template CSAPEX_PARAM_EXPORT ParameterBuilder ParameterFactory::declareParameterSet(const std::string&, const ParameterDescription&, const std::map<std::string, argument_type<void(T)>::type> &, const argument_type<void(T)>::type&); \
    template CSAPEX_PARAM_EXPORT ParameterBuilder ParameterFactory::declareValue(const std::string&, const ParameterDescription&, const argument_type<void(T)>::type&);


INSTANTIATE(bool)
INSTANTIATE(int)
INSTANTIATE(double)
INSTANTIATE(std::string)
INSTANTIATE((std::pair<int,int>))
INSTANTIATE((std::pair<double,double>))
INSTANTIATE((std::pair<std::string,bool>))
INSTANTIATE((std::vector<int>))
INSTANTIATE((std::vector<double>))
INSTANTIATE((std::vector<std::string>))


template
CSAPEX_PARAM_EXPORT ParameterBuilder ParameterFactory::declareRange<double>(const std::string& name,
                                                      const ParameterDescription& description,
                                                      double min, double max, double def, double step);
template
CSAPEX_PARAM_EXPORT ParameterBuilder ParameterFactory::declareRange<int>(const std::string& name,
                                                   const ParameterDescription& description,
                                                   int min, int max, int def, int step);

template
CSAPEX_PARAM_EXPORT ParameterBuilder ParameterFactory::declareInterval<double>(const std::string& name,
                                                      const ParameterDescription& description,
                                                      double min, double max, double def_min, double def_max, double step);
template
CSAPEX_PARAM_EXPORT ParameterBuilder ParameterFactory::declareInterval<int>(const std::string& name,
                                                   const ParameterDescription& description,
                                                   int min, int max, int def_min, int def_max, int step);
}
}
