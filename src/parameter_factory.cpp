/// HEADER
#include <utils_param/parameter_factory.h>

/// COMPONENT
#include <utils_param/color_parameter.h>
#include <utils_param/path_parameter.h>
#include <utils_param/interval_parameter.h>
#include <utils_param/trigger_parameter.h>
#include <utils_param/bitset_parameter.h>
#include <utils_param/range_parameter.h>
#include <utils_param/value_parameter.h>
#include <utils_param/string_list_parameter.h>
#include <utils_param/output_progress_parameter.h>

using namespace param;

std::unique_ptr<Parameter> ParameterFactory::makeEmpty(const std::string &type)
{
    std::string t = type;
    std::transform(t.begin(), t.end(), t.begin(), tolower);
    if(t == "range") {
        return std::unique_ptr<Parameter>(new RangeParameter);
    } else if(t == "interval") {
        return std::unique_ptr<Parameter>(new IntervalParameter);
    } else if(t == "value") {
        return std::unique_ptr<Parameter>(new ValueParameter);
    } else if(t == "set") {
        return std::unique_ptr<Parameter>(new SetParameter);
    } else if(t == "bitset") {
        return std::unique_ptr<Parameter>(new BitSetParameter);
    } else if(t == "path") {
        return std::unique_ptr<Parameter>(new PathParameter);
    } else if(t == "trigger") {
        return std::unique_ptr<Parameter>(new TriggerParameter);
    } else if(t == "string_list") {
        return std::unique_ptr<Parameter>(new StringListParameter);
    } else if(t == "color") {
        return std::unique_ptr<Parameter>(new ColorParameter);
    } else if(t == "progress") {
        return std::unique_ptr<Parameter>(new OutputProgressParameter);
    } else {
        throw std::runtime_error(std::string("illegal parameter type: ") + t);
    }
}

std::unique_ptr<Parameter> ParameterFactory::clone(const Parameter* param)
{
    std::string type = param->TYPE();
    std::unique_ptr<Parameter> r = makeEmpty(type);
    r->clone(*param);
    return r;
}
std::unique_ptr<Parameter> ParameterFactory::clone(const Parameter& param)
{
    return clone(&param);
}
std::unique_ptr<Parameter> ParameterFactory::clone(const std::unique_ptr<Parameter>& param)
{
    return clone(param.get());
}

std::unique_ptr<Parameter> ParameterFactory::clone(const std::shared_ptr<Parameter>& param)
{
    return clone(param.get());
}

std::unique_ptr<Parameter> ParameterFactory::declareParameterBitSet(const std::string &name, const ParameterDescription& description, const std::map<std::string, int> &set, int def)
{
    std::unique_ptr<BitSetParameter> result(new BitSetParameter(name, description));
    result->setBitSet(set);
    result->def_ = def;
    result->set<int>(def);

    return std::move(result);
}

std::unique_ptr<Parameter> ParameterFactory::declareParameterBitSet(const std::string &name, const std::map<std::string, int> &set, int def)
{
    return declareParameterBitSet(name, ParameterDescription(), set, def);
}

std::unique_ptr<Parameter> ParameterFactory::declareParameterBitSet(const std::string &name, const ParameterDescription& description, const std::map<std::string, std::pair<int, bool> > &set)
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

std::unique_ptr<Parameter> ParameterFactory::declareParameterBitSet(const std::string &name, const std::map<std::string, std::pair<int, bool> > &set)
{
    return ParameterFactory::declareParameterBitSet(name, ParameterDescription(), set);
}

std::unique_ptr<Parameter> ParameterFactory::declareBool(const std::string& name, const ParameterDescription& description, bool def)
{
    std::unique_ptr<ValueParameter> result(new ValueParameter(name, description));
    result->def_ = def;
    result->set<bool>(def);

    return std::move(result);
}

std::unique_ptr<Parameter> ParameterFactory::declareBool(const std::string &name, bool def)
{
    return declareBool(name, ParameterDescription(), def);
}

std::unique_ptr<Parameter> ParameterFactory::declareColorParameter(const std::string& name, const ParameterDescription& description, int r, int g, int b)
{
    std::unique_ptr<ColorParameter> result(new ColorParameter(name, description, r, g, b));
    return std::move(result);
}

std::unique_ptr<Parameter> ParameterFactory::declareColorParameter(const std::string& name, int r, int g, int b)
{
    return declareColorParameter(name, ParameterDescription(), r,g,b);
}

std::unique_ptr<Parameter> ParameterFactory::declareTrigger(const std::string& name, const ParameterDescription& description)
{
    std::unique_ptr<TriggerParameter> result(new TriggerParameter(name, description));
    return std::move(result);
}

std::unique_ptr<Parameter> ParameterFactory::declareTrigger(const std::string &name)
{
    return declareTrigger(name, ParameterDescription());
}

std::unique_ptr<Parameter> ParameterFactory::declarePath(const std::string& name, const ParameterDescription& description,
                                             bool is_file, const std::string& def, const std::string& filter, bool input, bool output)
{
    std::unique_ptr<PathParameter> result(new PathParameter(name, description, filter, is_file, input, output));
    result->set(def);

    return std::move(result);
}

std::unique_ptr<Parameter> ParameterFactory::declareFileInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, true, false);
}
std::unique_ptr<Parameter> ParameterFactory::declareFileInputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileInputPath(name, ParameterDescription(), def, filter);
}

std::unique_ptr<Parameter> ParameterFactory::declareFileOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, false, true);
}

std::unique_ptr<Parameter> ParameterFactory::declareFileOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileOutputPath(name, ParameterDescription(), def, filter);
}

std::unique_ptr<Parameter> ParameterFactory::declareFileInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, true, true);
}
std::unique_ptr<Parameter> ParameterFactory::declareFileInputOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileInputOutputPath(name, ParameterDescription(), def, filter);
}


std::unique_ptr<Parameter> ParameterFactory::declareDirectoryInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, true, false);
}

std::unique_ptr<Parameter> ParameterFactory::declareDirectoryInputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryInputPath(name, ParameterDescription(), def, filter);
}

std::unique_ptr<Parameter> ParameterFactory::declareDirectoryOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, false, true);
}
std::unique_ptr<Parameter> ParameterFactory::declareDirectoryOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryOutputPath(name, ParameterDescription(), def, filter);
}

std::unique_ptr<Parameter> ParameterFactory::declareDirectoryInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, true, true);
}
std::unique_ptr<Parameter> ParameterFactory::declareDirectoryInputOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryInputOutputPath(name, ParameterDescription(), def, filter);
}


std::unique_ptr<Parameter> ParameterFactory::declareText(const std::string& name, const ParameterDescription& description, const std::string& def)
{
    std::unique_ptr<ValueParameter> result(new ValueParameter(name, description));
    result->set(def);

    return std::move(result);
}

std::unique_ptr<Parameter> ParameterFactory::declareText(const std::string &name, const std::string &def)
{
    return declareText(name, ParameterDescription(""), def);
}

std::unique_ptr<Parameter> ParameterFactory::declareParameterStringSet(const std::string& name, const ParameterDescription& description,
                                                           const std::vector<std::string> & set,
                                                           const std::string &def)
{
    std::unique_ptr<SetParameter> result(new SetParameter(name, description));
    result->setSet(set);

    std::string v = def;
    if(!set.empty()) {
        if(v.empty()) {
            v = set[0];
        }
    }
    result->def_ = v;
    result->set<std::string>(v);

    return std::move(result);
}

std::unique_ptr<Parameter> ParameterFactory::declareParameterStringSet(const std::string& name, const std::vector<std::string> & set, const std::string& def)
{
    return declareParameterStringSet(name, ParameterDescription(), set, def);
}

template <typename T>
std::unique_ptr<Parameter> ParameterFactory::declareRange(const std::string& name,
                                   const ParameterDescription& description,
                                   T min, T max, T def, T step)
{
    BOOST_STATIC_ASSERT((boost::mpl::contains<RangeParameterTypes, T>::value));

    std::unique_ptr<RangeParameter> result(new RangeParameter(name, description));
    result->def_value_ = def;
    result->def_min_ = min;
    result->def_max_ = max;
    result->min_ = result->def_min_;
    result->max_ = result->def_max_;
    result->step_ = step;
    result->set<T>(def);

    return std::move(result);
}


template
std::unique_ptr<Parameter> ParameterFactory::declareRange<double>(const std::string& name,
                                                      const ParameterDescription& description,
                                                      double min, double max, double def, double step);
template
std::unique_ptr<Parameter> ParameterFactory::declareRange<int>(const std::string& name,
                                                   const ParameterDescription& description,
                                                   int min, int max, int def, int step);




template <typename T>
std::unique_ptr<Parameter> ParameterFactory::declareInterval(const std::string& name,
                                                 const ParameterDescription& description,
                                                 T min, T max, T def_min, T def_max, T step)
{
    BOOST_STATIC_ASSERT((boost::mpl::contains<IntervalParameterTypes, T>::value));

    std::unique_ptr<IntervalParameter> result(new IntervalParameter(name, description));
    result->def_ = std::make_pair(def_min, def_max);
    result->min_ = min;
    result->max_ = max;
    result->step_ = step;
    result->values_ = std::make_pair(def_min, def_max);

    result->set<std::pair<T,T> >(std::make_pair(def_min, def_max));

    return std::move(result);
}


template
std::unique_ptr<Parameter> ParameterFactory::declareInterval<double>(const std::string& name,
                                                      const ParameterDescription& description,
                                                      double min, double max, double def_min, double def_max, double step);
template
std::unique_ptr<Parameter> ParameterFactory::declareInterval<int>(const std::string& name,
                                                   const ParameterDescription& description,
                                                   int min, int max, int def_min, int def_max, int step);


std::unique_ptr<Parameter> ParameterFactory::declareOutputProgress(const std::string &name, const ParameterDescription &description)
{
    std::unique_ptr<Parameter> result(new OutputProgressParameter(name, description));
    return std::move(result);
}
