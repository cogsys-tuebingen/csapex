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

Parameter::Ptr ParameterFactory::declareParameterBitSet(const std::string &name, const ParameterDescription& description, const std::map<std::string, int> &set, int def)
{
    BitSetParameter::Ptr result(new BitSetParameter(name, description));
    result->setBitSet(set);
    result->def_ = def;
    result->set<int>(def);

    return result;
}

Parameter::Ptr ParameterFactory::declareParameterBitSet(const std::string &name, const std::map<std::string, int> &set, int def)
{
    return declareParameterBitSet(name, ParameterDescription(), set, def);
}

Parameter::Ptr ParameterFactory::declareParameterBitSet(const std::string &name, const ParameterDescription& description, const std::map<std::string, std::pair<int, bool> > &set)
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

Parameter::Ptr ParameterFactory::declareParameterBitSet(const std::string &name, const std::map<std::string, std::pair<int, bool> > &set)
{
    return ParameterFactory::declareParameterBitSet(name, ParameterDescription(), set);
}

Parameter::Ptr ParameterFactory::declare(const std::string& name, const char* def)
{
    return declare(name, std::string(def));
}

Parameter::Ptr ParameterFactory::declare(const std::string& name, bool def)
{
    return declareBool(name, def);
}


Parameter::Ptr ParameterFactory::declareBool(const std::string& name, const ParameterDescription& description, bool def)
{
    ValueParameter::Ptr result(new ValueParameter(name, description));
    result->def_ = def;
    result->set<bool>(def);

    return result;
}

Parameter::Ptr ParameterFactory::declareBool(const std::string &name, bool def)
{
    return declareBool(name, ParameterDescription(), def);
}

Parameter::Ptr ParameterFactory::declareColorParameter(const std::string& name, const ParameterDescription& description, int r, int g, int b)
{
    ColorParameter::Ptr result(new ColorParameter(name, description, r, g, b));
    return result;
}

Parameter::Ptr ParameterFactory::declareColorParameter(const std::string& name, int r, int g, int b)
{
    return declareColorParameter(name, ParameterDescription(), r,g,b);
}

Parameter::Ptr ParameterFactory::declareTrigger(const std::string& name, const ParameterDescription& description)
{
    TriggerParameter::Ptr result(new TriggerParameter(name, description));
    return result;
}

Parameter::Ptr ParameterFactory::declareTrigger(const std::string &name)
{
    return declareTrigger(name, ParameterDescription());
}

Parameter::Ptr ParameterFactory::declarePath(const std::string& name, const ParameterDescription& description,
                                             bool is_file, const std::string& def, const std::string& filter, bool input, bool output)
{
    PathParameter::Ptr result(new PathParameter(name, description, filter, is_file, input, output));
    result->set(def);

    return result;
}

Parameter::Ptr ParameterFactory::declareFileInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, true, false);
}
Parameter::Ptr ParameterFactory::declareFileInputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileInputPath(name, ParameterDescription(), def, filter);
}

Parameter::Ptr ParameterFactory::declareFileOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, false, true);
}

Parameter::Ptr ParameterFactory::declareFileOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileOutputPath(name, ParameterDescription(), def, filter);
}

Parameter::Ptr ParameterFactory::declareFileInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, true, true);
}
Parameter::Ptr ParameterFactory::declareFileInputOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileInputOutputPath(name, ParameterDescription(), def, filter);
}


Parameter::Ptr ParameterFactory::declareDirectoryInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, true, false);
}

Parameter::Ptr ParameterFactory::declareDirectoryInputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryInputPath(name, ParameterDescription(), def, filter);
}

Parameter::Ptr ParameterFactory::declareDirectoryOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, false, true);
}
Parameter::Ptr ParameterFactory::declareDirectoryOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryOutputPath(name, ParameterDescription(), def, filter);
}

Parameter::Ptr ParameterFactory::declareDirectoryInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, true, true);
}
Parameter::Ptr ParameterFactory::declareDirectoryInputOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryInputOutputPath(name, ParameterDescription(), def, filter);
}


Parameter::Ptr ParameterFactory::declareText(const std::string& name, const ParameterDescription& description, const std::string& def)
{
    ValueParameter::Ptr result(new ValueParameter(name, description));
    result->set(def);

    return result;
}

Parameter::Ptr ParameterFactory::declareText(const std::string &name, const std::string &def)
{
    return declareText(name, ParameterDescription(""), def);
}

Parameter::Ptr ParameterFactory::declareParameterStringSet(const std::string& name, const ParameterDescription& description,
                                                           const std::vector<std::string> & set,
                                                           const std::string &def)
{
    SetParameter::Ptr result(new SetParameter(name, description));
    result->setSet(set);

    if(!set.empty()) {
        std::string v = def;
        if(v.empty()) {
            v = set[0];
        }
        result->def_ = v;
        result->set<std::string>(v);
    }

    return result;
}

Parameter::Ptr ParameterFactory::declareParameterStringSet(const std::string& name, const std::vector<std::string> & set, const std::string& def)
{
    return declareParameterStringSet(name, ParameterDescription(), set, def);
}
