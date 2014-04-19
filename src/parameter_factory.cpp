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
    std::string t = type;
    std::transform(t.begin(), t.end(), t.begin(), tolower);
    if(t == "range") {
        return Parameter::Ptr(new RangeParameter);
    } else if(t == "interval") {
        return Parameter::Ptr(new IntervalParameter);
    } else if(t == "value") {
        return Parameter::Ptr(new ValueParameter);
    } else if(t == "set") {
        return Parameter::Ptr(new SetParameter);
    } else if(t == "bitset") {
        return Parameter::Ptr(new BitSetParameter);
    } else if(t == "path") {
        return Parameter::Ptr(new PathParameter);
    } else if(t == "trigger") {
        return Parameter::Ptr(new TriggerParameter);
    } else if(t == "color") {
        return Parameter::Ptr(new ColorParameter);
    } else {
        throw std::runtime_error(std::string("illegal parameter type: ") + t);
    }
}

Parameter::Ptr ParameterFactory::clone(const Parameter::Ptr& param)
{
    std::string type = param->TYPE();
    Parameter::Ptr r = makeEmpty(type);
    r->clone(*param);
    return r;
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

Parameter::Ptr ParameterFactory::declarePath(const std::string& name, bool is_file, const std::string& def, const std::string& filter, bool input, bool output)
{
    PathParameter::Ptr result(new PathParameter(name, filter, is_file, input, output));
    result->set(def);

    return result;
}

Parameter::Ptr ParameterFactory::declareFileInputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declarePath(name, true, def, filter, true, false);
}

Parameter::Ptr ParameterFactory::declareFileOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declarePath(name, true, def, filter, false, true);
}

Parameter::Ptr ParameterFactory::declareFileInputOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declarePath(name, true, def, filter, true, true);
}


Parameter::Ptr ParameterFactory::declareDirectoryInputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declarePath(name, false, def, filter, true, false);
}

Parameter::Ptr ParameterFactory::declareDirectoryOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declarePath(name, false, def, filter, false, true);
}

Parameter::Ptr ParameterFactory::declareDirectoryInputOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declarePath(name, false, def, filter, true, true);
}


Parameter::Ptr ParameterFactory::declareText(const std::string& name, const std::string& def)
{
    ValueParameter::Ptr result(new ValueParameter(name));
    result->set(def);

    return result;
}
