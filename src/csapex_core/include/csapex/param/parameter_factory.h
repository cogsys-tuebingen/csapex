#ifndef PARAMETER_FACTORY_H
#define PARAMETER_FACTORY_H

/// COMPONENT
#include <csapex/param/parameter.h>
#include <csapex/param/parameter_builder.h>
#include <csapex/param/parameter_description.h>
#include <csapex_core/csapex_param_export.h>

/// PROJECT
#include <csapex/utility/singleton.hpp>

/// SYSTEM
#include <type_traits>

namespace csapex
{
namespace param
{
namespace factory
{
/**
 * @brief makeEmpty creates an empty parameter pointer
 * @param type
 * @return
 */
ParameterBuilder makeEmpty(const std::string& type);

/**
 * @brief registerParameterType registers a new parameter type for registration
 * @param type the type name
 * @param constructor construction function
 */
void registerParameterType(const std::string& type, std::function<ParameterBuilder()> constructor);

/**
 * @brief registerParameterType deregisters a new parameter type for registration
 * @param type the type name
 */
void deregisterParameterType(const std::string& type);

/**
 * @brief clone duplicates a parameter deeply
 * @param param
 * @return
 */
ParameterBuilder clone(const Parameter* param);
ParameterBuilder clone(const Parameter& param);
ParameterBuilder clone(const std::shared_ptr<Parameter>& param);

/**
 * @brief declareRange
 * @param name
 * @param description
 * @param min
 * @param max
 * @param def
 * @param step
 * @return
 */
template <typename T>
ParameterBuilder declareRange(const std::string& name, const ParameterDescription& description, T min, T max, T def, T step);

template <typename T>
ParameterBuilder declareRange(const std::string& name, T min, T max, T def, T step)
{
    return declareRange(name, ParameterDescription(), min, max, def, step);
}

/**
 * @brief declareInterval
 * @param name
 * @param description
 * @param min
 * @param max
 * @param def_min
 * @param def_max
 * @param step
 * @return
 */
template <typename T>
ParameterBuilder declareInterval(const std::string& name, const ParameterDescription& description, T min, T max, T def_min, T def_max, T step);

template <typename T>
ParameterBuilder declareInterval(const std::string& name, T min, T max, T def_min, T def_max, T step)
{
    return declareInterval(name, ParameterDescription(), min, max, def_min, def_max, step);
}

/**
 * @brief declareBool
 * @param name
 * @param def
 * @return
 */
ParameterBuilder declareBool(const std::string& name, bool def);
ParameterBuilder declareBool(const std::string& name, const ParameterDescription& description, bool def);

/**
 * @brief declareText
 * @param name
 * @param description
 * @param def
 * @return
 */
ParameterBuilder declareText(const std::string& name, const ParameterDescription& description, const std::string& def);
ParameterBuilder declareText(const std::string& name, const std::string& def);

/**
 * @brief declarePath
 * @param name
 * @param description
 * @param is_file
 * @param def
 * @param filter
 * @param input
 * @param output
 * @return
 */
ParameterBuilder declarePath(const std::string& name, const ParameterDescription& description, bool is_file, const std::string& def, const std::string& filter = "", bool input = false,
                             bool output = false);

/**
 * @brief declareFileInputPath
 * @param name
 * @param description
 * @param def
 * @param filter
 * @return
 */
ParameterBuilder declareFileInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
ParameterBuilder declareFileInputPath(const std::string& name, const std::string& def, const std::string& filter = "");

/**
 * @brief declareFileOutputPath
 * @param name
 * @param description
 * @param def
 * @param filter
 * @return
 */
ParameterBuilder declareFileOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
ParameterBuilder declareFileOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");

/**
 * @brief declareFileInputOutputPath
 * @param name
 * @param description
 * @param def
 * @param filter
 * @return
 */
ParameterBuilder declareFileInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
ParameterBuilder declareFileInputOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");

/**
 * @brief declareDirectoryInputPath
 * @param name
 * @param description
 * @param def
 * @param filter
 * @return
 */
ParameterBuilder declareDirectoryInputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
ParameterBuilder declareDirectoryInputPath(const std::string& name, const std::string& def, const std::string& filter = "");

/**
 * @brief declareDirectoryOutputPath
 * @param name
 * @param description
 * @param def
 * @param filter
 * @return
 */
ParameterBuilder declareDirectoryOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
ParameterBuilder declareDirectoryOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");

/**
 * @brief declareDirectoryInputOutputPath
 * @param name
 * @param description
 * @param def
 * @param filter
 * @return
 */
ParameterBuilder declareDirectoryInputOutputPath(const std::string& name, const ParameterDescription& description, const std::string& def, const std::string& filter = "");
ParameterBuilder declareDirectoryInputOutputPath(const std::string& name, const std::string& def, const std::string& filter = "");

/**
 * @brief declareTrigger
 * @param name
 * @param description
 * @return
 */
ParameterBuilder declareTrigger(const std::string& name, const ParameterDescription& description);
ParameterBuilder declareTrigger(const std::string& name);

/**
 * @brief declareColorParameter
 * @param name
 * @param description
 * @param r
 * @param g
 * @param b
 * @return
 */
ParameterBuilder declareColorParameter(const std::string& name, const ParameterDescription& description, int r, int g, int b);
ParameterBuilder declareColorParameter(const std::string& name, int r, int g, int b);

/**
 * @brief declareStringList
 * @param name
 * @param description
 * @return
 */
ParameterBuilder declareStringList(const std::string& name, const ParameterDescription& description, const std::vector<std::string>& list);
ParameterBuilder declareStringList(const std::string& name, const std::vector<std::string>& list);

namespace detail
{
template <typename T>
ParameterBuilder declareParameterSetImpl(const std::string& name, const ParameterDescription& description, const std::map<std::string, T>& set_values, const T& def);
template <typename T>
ParameterBuilder declareParameterSetImpl(const std::string& name, const std::map<std::string, T>& set, const T& def)
{
    return declareParameterSetImpl(name, ParameterDescription(), set, def);
}
}  // namespace detail

/**
 * @brief declareParameterSet
 * @param name
 * @param description
 * @param set
 * @param def default value
 * @return
 */
template <typename T, typename std::enable_if<!std::is_enum<T>::value, int>::type = 0>
ParameterBuilder declareParameterSet(const std::string& name, const ParameterDescription& description, const std::map<std::string, T>& set_values, const T& def)
{
    return detail::declareParameterSetImpl(name, description, set_values, def);
}

template <typename T, typename std::enable_if<std::is_enum<T>::value, int>::type = 0>
ParameterBuilder declareParameterSet(const std::string& name, const ParameterDescription& description, const std::map<std::string, T>& set_values, const T& def)
{
    std::map<std::string, int> int_map;
    for (const auto& entry : set_values) {
        int_map[entry.first] = static_cast<int>(entry.second);
    }
    return detail::declareParameterSetImpl(name, description, int_map, static_cast<int>(def));
}

template <typename T, typename std::enable_if<!std::is_enum<T>::value, int>::type = 0>
ParameterBuilder declareParameterSet(const std::string& name, const std::map<std::string, T>& set, const T& def)
{
    return declareParameterSet(name, ParameterDescription(), set, def);
}

template <typename T, typename std::enable_if<std::is_enum<T>::value, int>::type = 0>
ParameterBuilder declareParameterSet(const std::string& name, const std::map<std::string, T>& set, const T& def)
{
    return declareParameterSet(name, ParameterDescription(), set, def);
}

/**
 * @brief declareParameterStringSet
 * @param name
 * @param description
 * @param set
 * @param def default value
 * @return
 */
ParameterBuilder declareParameterStringSet(const std::string& name, const ParameterDescription& description, const std::vector<std::string>& set, const std::string& def = "");
ParameterBuilder declareParameterStringSet(const std::string& name, const std::vector<std::string>& set, const std::string& def = "");

/**
 * @brief declareParameterBitSet
 * @param name
 * @param description
 * @param set
 * @param def
 * @return
 */
ParameterBuilder declareParameterBitSet(const std::string& name, const ParameterDescription& description, const std::map<std::string, int>& set, int def = 0);
ParameterBuilder declareParameterBitSet(const std::string& name, const std::map<std::string, int>& set, int def = 0);

/**
 * @brief declareParameterBitSet
 * @param name
 * @param description
 * @param set
 * @return
 */
ParameterBuilder declareParameterBitSet(const std::string& name, const ParameterDescription& description, const std::map<std::string, std::pair<int, bool>>& set);
ParameterBuilder declareParameterBitSet(const std::string& name, const std::map<std::string, std::pair<int, bool>>& set);

/**
 * @brief declareValue
 * @param name
 * @param description
 * @param def
 * @return
 */
template <typename T>
ParameterBuilder declareValue(const std::string& name, const ParameterDescription& description, const T& def);

template <typename T>
ParameterBuilder declareValue(const std::string& name, const T& def)
{
    return declareValue(name, ParameterDescription(), def);
}
inline ParameterBuilder declareValue(const std::string& name, const char* def)
{
    return declareValue(name, ParameterDescription(), std::string(def));
}

/**
 * @brief declareOutputProgress
 * @param name
 * @param description
 * @return
 */
ParameterBuilder declareOutputProgress(const std::string& name, const ParameterDescription& description = ParameterDescription(""));

/**
 * @brief declareOutputText
 * @param name
 * @param description
 * @return
 */
ParameterBuilder declareOutputText(const std::string& name, const ParameterDescription& description = ParameterDescription(""));

class ParameterFactory : public Singleton<ParameterFactory>
{
public:
    template <typename T>
    struct ParameterConstructorRegistered
    {
        ParameterConstructorRegistered()
        {
            factory::registerParameterType(param::serializationName<T>(), []() { return ParameterBuilder(std::move(std::make_shared<T>())); });
        }

        ~ParameterConstructorRegistered()
        {
            factory::deregisterParameterType(param::serializationName<T>());
        }
    };

public:
    ParameterBuilder makeEmpty(const std::string& type);
    void registerParameterType(const std::string& type, std::function<ParameterBuilder()> constructor);
    void deregisterParameterType(const std::string& type);

private:
    std::map<std::string, std::function<ParameterBuilder()>> type_to_constructor;
};

}  // namespace factory

// backwards compatibility
namespace ParameterFactory = factory;

}  // namespace param
}  // namespace csapex

#endif  // PARAMETER_FACTORY_H
