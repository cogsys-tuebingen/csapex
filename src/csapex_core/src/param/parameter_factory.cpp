/// HEADER
#include <csapex/param/parameter_factory.h>

/// COMPONENT
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

void factory::registerParameterType(const std::string& type, std::function<ParameterBuilder()> constructor)
{
    // std::cout << "register constructor " << type << std::endl;
    ParameterFactory& i = ParameterFactory::instance();
    i.registerParameterType(type, constructor);
}
void factory::deregisterParameterType(const std::string& type)
{
    // std::cout << "deregister constructor " << type << std::endl;
    ParameterFactory& i = ParameterFactory::instance();
    i.deregisterParameterType(type);
}

void factory::ParameterFactory::registerParameterType(const std::string& type, std::function<ParameterBuilder()> constructor)
{
    std::map<std::string, std::function<ParameterBuilder()>>::const_iterator it = type_to_constructor.find(type);

    type_to_constructor.insert(std::make_pair(type, constructor));
}
void factory::ParameterFactory::deregisterParameterType(const std::string& type)
{
    auto pos = type_to_constructor.find(type);
    if (pos != type_to_constructor.end()) {
        type_to_constructor.erase(pos);
    }
}

ParameterBuilder factory::ParameterFactory::makeEmpty(const std::string& type)
{
    if (type_to_constructor.find(type) == type_to_constructor.end()) {
        throw std::runtime_error(std::string("cannot create parameter, no such type (") + type + ")");
    }

    return type_to_constructor[type]();
}

ParameterBuilder factory::makeEmpty(const std::string& type)
{
    std::string t = type;
    std::transform(t.begin(), t.end(), t.begin(), tolower);
    return ParameterFactory::instance().makeEmpty(type);
}

ParameterBuilder factory::clone(const Parameter* param)
{
    return param->cloneAs<Parameter>();
}
ParameterBuilder factory::clone(const Parameter& param)
{
    return clone(&param);
}
ParameterBuilder factory::clone(const std::shared_ptr<Parameter>& param)
{
    return clone(param.get());
}

ParameterBuilder factory::declareParameterBitSet(const std::string& name, const ParameterDescription& description, const std::map<std::string, int>& set, int def)
{
    std::shared_ptr<BitSetParameter> result(new BitSetParameter(name, description, def));
    result->setBitSet(set);
    result->set<int>(def);

    return ParameterBuilder(std::move(result));
}

ParameterBuilder factory::declareParameterBitSet(const std::string& name, const std::map<std::string, int>& set, int def)
{
    return declareParameterBitSet(name, ParameterDescription(), set, def);
}

ParameterBuilder factory::declareParameterBitSet(const std::string& name, const ParameterDescription& description, const std::map<std::string, std::pair<int, bool>>& set)
{
    std::map<std::string, int> raw_set;
    int def = 0;

    for (std::map<std::string, std::pair<int, bool>>::const_iterator it = set.begin(); it != set.end(); ++it) {
        raw_set[it->first] = it->second.first;
        if (it->second.second) {
            def += it->second.first;
        }
    }
    return declareParameterBitSet(name, description, raw_set, def);
}

ParameterBuilder factory::declareParameterBitSet(const std::string& name, const std::map<std::string, std::pair<int, bool>>& set)
{
    return factory::declareParameterBitSet(name, ParameterDescription(), set);
}

ParameterBuilder factory::declareBool(const std::string& name, const ParameterDescription& description, bool def)
{
    std::shared_ptr<ValueParameter> result(new ValueParameter(name, description, def));
    result->set<bool>(def);

    return ParameterBuilder(std::move(result));
}

ParameterBuilder factory::declareBool(const std::string& name, bool def)
{
    return declareBool(name, ParameterDescription(), def);
}

ParameterBuilder factory::declareColorParameter(const std::string& name, const ParameterDescription& description, int r, int g, int b)
{
    std::shared_ptr<ColorParameter> result(new ColorParameter(name, description, r, g, b));
    return ParameterBuilder(std::move(result));
}

ParameterBuilder factory::declareColorParameter(const std::string& name, int r, int g, int b)
{
    return declareColorParameter(name, ParameterDescription(), r, g, b);
}

ParameterBuilder factory::declareTrigger(const std::string& name, const ParameterDescription& description)
{
    std::shared_ptr<TriggerParameter> result(new TriggerParameter(name, description));
    return ParameterBuilder(std::move(result));
}

ParameterBuilder factory::declareStringList(const std::string& name, const ParameterDescription& description, const std::vector<std::string>& list)
{
    std::shared_ptr<StringListParameter> result(new StringListParameter(name, description));
    result->set(list);
    return ParameterBuilder(std::move(result));
}
ParameterBuilder factory::declareStringList(const std::string& name, const std::vector<std::string>& list)
{
    return declareStringList(name, ParameterDescription(), list);
}

ParameterBuilder factory::declareTrigger(const std::string& name)
{
    return declareTrigger(name, ParameterDescription());
}

ParameterBuilder factory::declarePath(const std::string& name, const ParameterDescription& description, bool is_file, const std::string& def, const std::string& filter, bool input, bool output)
{
    std::shared_ptr<PathParameter> result(new PathParameter(name, description, filter, is_file, input, output));
    result->set(def);

    return ParameterBuilder(std::move(result));
}

ParameterBuilder factory::declareFileInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, true, false);
}
ParameterBuilder factory::declareFileInputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileInputPath(name, ParameterDescription(), def, filter);
}

ParameterBuilder factory::declareFileOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, false, true);
}

ParameterBuilder factory::declareFileOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileOutputPath(name, ParameterDescription(), def, filter);
}

ParameterBuilder factory::declareFileInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, true, def, filter, true, true);
}
ParameterBuilder factory::declareFileInputOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareFileInputOutputPath(name, ParameterDescription(), def, filter);
}

ParameterBuilder factory::declareDirectoryInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, true, false);
}

ParameterBuilder factory::declareDirectoryInputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryInputPath(name, ParameterDescription(), def, filter);
}

ParameterBuilder factory::declareDirectoryOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, false, true);
}
ParameterBuilder factory::declareDirectoryOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryOutputPath(name, ParameterDescription(), def, filter);
}

ParameterBuilder factory::declareDirectoryInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter)
{
    return declarePath(name, description, false, def, filter, true, true);
}
ParameterBuilder factory::declareDirectoryInputOutputPath(const std::string& name, const std::string& def, const std::string& filter)
{
    return declareDirectoryInputOutputPath(name, ParameterDescription(), def, filter);
}

ParameterBuilder factory::declareText(const std::string& name, const ParameterDescription& description, const std::string& def)
{
    std::shared_ptr<ValueParameter> result(new ValueParameter(name, description));
    result->set(def);

    return ParameterBuilder(std::move(result));
}

ParameterBuilder factory::declareText(const std::string& name, const std::string& def)
{
    return declareText(name, ParameterDescription(""), def);
}

ParameterBuilder factory::declareParameterStringSet(const std::string& name, const ParameterDescription& description, const std::vector<std::string>& set, const std::string& def)
{
    std::shared_ptr<SetParameter> result;
    if (!set.empty()) {
        result.reset(new SetParameter(name, description, def));
    } else {
        result.reset(new SetParameter(name, description));
    }

    result->setSet(set);

    if (!set.empty()) {
        std::string v = def;
        if (v.empty()) {
            v = set[0];
        }
        result->set<std::string>(v);
    }

    return ParameterBuilder(std::move(result));
}

ParameterBuilder factory::declareParameterStringSet(const std::string& name, const std::vector<std::string>& set, const std::string& def)
{
    return declareParameterStringSet(name, ParameterDescription(), set, def);
}

ParameterBuilder factory::declareOutputProgress(const std::string& name, const ParameterDescription& description)
{
    std::shared_ptr<OutputProgressParameter> result(new OutputProgressParameter(name, description));
    return ParameterBuilder(std::move(result));
}

ParameterBuilder factory::declareOutputText(const std::string& name, const ParameterDescription& description)
{
    std::shared_ptr<OutputTextParameter> result(new OutputTextParameter(name, description));
    return ParameterBuilder(std::move(result));
}

namespace csapex
{
namespace param
{
template <typename T>
ParameterBuilder factory::declareRange(const std::string& name, const ParameterDescription& description, T min, T max, T def, T step)
{
    BOOST_STATIC_ASSERT((boost::mpl::contains<RangeParameterTypes, T>::value));

    step = param::range::limitStep<T>(min, max, step);

    std::shared_ptr<RangeParameter> result(new RangeParameter(name, description, def, min, max, step));
    result->set<T>(def);

    return ParameterBuilder(std::move(result));
}

template <typename T>
ParameterBuilder factory::declareInterval(const std::string& name, const ParameterDescription& description, T min, T max, T def_min, T def_max, T step)
{
    BOOST_STATIC_ASSERT((boost::mpl::contains<IntervalParameterTypes, T>::value));

    std::shared_ptr<IntervalParameter> result(new IntervalParameter(name, description, std::make_pair(def_min, def_max), min, max, step));

    result->set<std::pair<T, T>>(std::make_pair(def_min, def_max));

    return ParameterBuilder(std::move(result));
}

template <typename T>
ParameterBuilder factory::detail::declareParameterSetImpl(const std::string& name, const ParameterDescription& description, const std::map<std::string, T>& set_values, const T& def)
{
    std::unique_ptr<SetParameter> result(new SetParameter(name, description, def));
    result->setSet(set_values);
    if (!set_values.empty()) {
        result->set<T>(def);
    }

    return ParameterBuilder(std::move(result));
}
template <typename T>
ParameterBuilder factory::declareValue(const std::string& name, const ParameterDescription& description, const T& def)
{
    std::unique_ptr<ValueParameter> result(new ValueParameter(name, description, def));
    result->set<T>(def);

    return ParameterBuilder(std::move(result));
}

/// EXPLICIT INSTANTIATON

namespace
{
template <typename T>
struct argument_type;
template <typename T, typename U>
struct argument_type<T(U)>
{
    typedef U type;
};
}  // namespace

#define INSTANTIATE(T)                                                                                                                                                                                 \
    template ParameterBuilder factory::detail::declareParameterSetImpl(const std::string&, const ParameterDescription&, const std::map<std::string, argument_type<void(T)>::type>&,                    \
                                                                       const argument_type<void(T)>::type&);                                                                                           \
    template ParameterBuilder factory::declareValue(const std::string&, const ParameterDescription&, const argument_type<void(T)>::type&);

INSTANTIATE(bool)
INSTANTIATE(int)
INSTANTIATE(double)
INSTANTIATE(long)
INSTANTIATE(std::string)
INSTANTIATE((std::pair<int, int>))
INSTANTIATE((std::pair<double, double>))
INSTANTIATE((std::pair<std::string, bool>))
INSTANTIATE((std::vector<int>))
INSTANTIATE((std::vector<double>))
INSTANTIATE((std::vector<std::string>))

template ParameterBuilder factory::declareRange<double>(const std::string& name, const ParameterDescription& description, double min, double max, double def, double step);
template ParameterBuilder factory::declareRange<int>(const std::string& name, const ParameterDescription& description, int min, int max, int def, int step);

template ParameterBuilder factory::declareInterval<double>(const std::string& name, const ParameterDescription& description, double min, double max, double def_min, double def_max, double step);
template ParameterBuilder factory::declareInterval<int>(const std::string& name, const ParameterDescription& description, int min, int max, int def_min, int def_max, int step);
}  // namespace param
}  // namespace csapex
