/// HEADER
#include <utils_param/parameter_factory.h>

/// COMPONENT
#include <utils_param/color_parameter.h>
#include <utils_param/path_parameter.h>
#include <utils_param/interval_parameter.h>
#include <utils_param/trigger_parameter.h>
#include <utils_param/bitset_parameter.h>

using namespace param;

Parameter::Ptr ParameterFactory::makeEmpty(const std::string &type)
{
    if(type == "range") {
        return Parameter::Ptr(new RangeParameter);
    } else if(type == "interval") {
        return Parameter::Ptr(new IntervalParameter);
    } else if(type == "value") {
        return Parameter::Ptr(new ValueParameter);
    } else if(type == "set") {
        return Parameter::Ptr(new SetParameter);
    } else if(type == "bitset") {
        return Parameter::Ptr(new BitSetParameter);
    } else if(type == "path") {
        return Parameter::Ptr(new PathParameter);
    } else if(type == "trigger") {
        return Parameter::Ptr(new TriggerParameter);
    } else if(type == "color") {
        return Parameter::Ptr(new ColorParameter);
    } else {
        throw std::runtime_error(std::string("illegal parameter type: ") + type);
    }
}

Parameter::Ptr ParameterFactory::declareParameterBitSet(const std::string &name, const std::map<std::string, int> &set, int def)
{
    BitSetParameter::Ptr result(new BitSetParameter(name));
    result->setBitSet(set);
    result->def_ = def;
    result->set<int>(def);

    return result;
}


Parameter::Ptr ParameterFactory::declareParameterBitSet(const std::string &name, const std::map<std::string, std::pair<int, bool> > &set)
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
    return declareParameterBitSet(name, raw_set, def);
}

Parameter::Ptr ParameterFactory::declare(const std::string& name, const char* def)
{
    return declare(name, std::string(def));
}

Parameter::Ptr ParameterFactory::declare(const std::string& name, bool def)
{
    return declareBool(name, def);
}


Parameter::Ptr ParameterFactory::declareBool(const std::string& name, bool def)
{
    ValueParameter::Ptr result(new ValueParameter(name));
    result->def_ = def;
    result->set<bool>(def);

    return result;
}

Parameter::Ptr ParameterFactory::declareColorParameter(const std::string& name, int r, int g, int b)
{
    ColorParameter::Ptr result(new ColorParameter(name, r, g, b));
    return result;
}

Parameter::Ptr ParameterFactory::declareTrigger(const std::string& name)
{
    TriggerParameter::Ptr result(new TriggerParameter(name));
    return result;
}

Parameter::Ptr ParameterFactory::declarePath(const std::string& name, const std::string& def)
{
    PathParameter::Ptr result(new PathParameter(name));
    result->set(def);

    return result;
}


Parameter::Ptr ParameterFactory::declareText(const std::string& name, const std::string& def)
{
    ValueParameter::Ptr result(new ValueParameter(name));
    result->set(def);

    return result;
}
